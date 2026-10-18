//! `klipperx mainsail` — serve the Mainsail web UI, layered so that where the
//! site comes from can change without changing this code.
//!
//! Mainsail is a Vite single-page application: the checkout under
//! `third_party/mainsail` is its *source*, and what a browser can use is the
//! `dist/` directory `npm ci && npm run build` produces from it. This subcommand
//! is the missing middle: it finds a built site, makes one if the checkout layer
//! has none, and serves it over HTTP.
//!
//! Three layers, the first that applies wins:
//!
//! 1. `--dir <DIR>` — a site that is already built. Served as it is, never
//!    built, never second-guessed.
//! 2. **Embedded** — the site compiled into the binary by `build.rs` when the
//!    `embedded-mainsail` feature was on and `third_party/mainsail/dist` (or
//!    `KLIPPERX_MAINSAIL_DIST`) existed at build time. Off by default, because
//!    embedding costs the site's bytes in every build.
//! 3. **The checkout's `dist/`** — built with `npm ci && npm run build` when it
//!    is missing or older than `package-lock.json`, unless `--no-build` says
//!    never to run npm. `--rebuild` forces this layer and forces the build.
//!
//! Whichever layer answered, the site needs one file it cannot ship: `config.json`,
//! which says which Moonraker to talk to. For layers 1 and 3 it is written into
//! the served directory when absent; for layer 2, where there is nowhere to
//! write, an equivalent file is served from memory.
//!
//! The HTTP side is deliberately plain: HTTP/1.1, one request per connection,
//! `GET` and `HEAD`, no keep-alive. A path that names a file under the site is
//! served with its type, a path that names nothing falls back to `index.html`
//! (the application routes in the browser, so any URL is a page), and a path
//! that tries to climb out with `..` — spelled plainly or percent-encoded — is
//! refused.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use clap::Args;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

use crate::ui::command::{self, CommandRunner, Invocation, SystemRunner};
use crate::ui::{mainsail_checkout, mainsail_dist};

/// The port the site is served on by default.
pub const DEFAULT_PORT: u16 = 8080;

/// The address the site is served on by default: Mainsail is normally opened
/// from another machine on the same network.
pub const DEFAULT_BIND: &str = "0.0.0.0";

/// The port Moonraker listens on, written into the site's `config.json`.
pub const MOONRAKER_PORT: u16 = 7125;

/// The largest request head accepted: a request line, some headers, and room to
/// spare. Nothing Mainsail sends is near it.
const MAX_REQUEST_BYTES: usize = 8 * 1024;

/// The source order, spelled out in `--help`: an operator should be able to tell
/// which directory (or which copy in the binary) a run will serve.
const SITE_SOURCE_HELP: &str = "\
SITE SOURCE (the first that applies is used):
  1. --dir <DIR>                    a site that is already built; never built
  2. embedded in this binary        only when built with --features embedded-mainsail
                                    and a dist/ existed at build time
  3. <repo>/third_party/mainsail/dist
                                    built with `npm ci && npm run build` when it is
                                    missing or older than package-lock.json
                                    (KLIPPERX_MAINSAIL_DIST moves this layer elsewhere)

--no-build serves what exists and never runs npm; --rebuild always builds the
checkout's site, so the embedded copy is not used. The served path is printed
when the service starts.";

/// Serve the Mainsail site.
#[derive(Args, Debug, Clone)]
#[command(after_help = SITE_SOURCE_HELP)]
pub struct MainsailArgs {
    /// TCP port to serve the site on
    #[arg(long, default_value_t = DEFAULT_PORT)]
    pub port: u16,

    /// Address to serve on (0.0.0.0 for the whole network)
    #[arg(long, default_value = DEFAULT_BIND)]
    pub bind: String,

    /// Serve a site that is already built (skips the embedded copy and the checkout)
    ///
    /// The directory must contain `index.html`: `--dir` is served as found and
    /// nothing is built into it.
    #[arg(long, value_name = "DIR")]
    pub dir: Option<PathBuf>,

    /// Serve what is there; never run npm
    ///
    /// With this, a site that is not built is an error naming the command that
    /// would have built it.
    #[arg(long, conflicts_with = "rebuild")]
    pub no_build: bool,

    /// Rebuild the checkout's site even if it looks current
    ///
    /// Forces the checkout layer — an embedded copy is not used — and runs
    /// `npm ci && npm run build` even when `dist/` is newer than the lock file.
    #[arg(long)]
    pub rebuild: bool,
}

/// Run `klipperx mainsail`: resolve and prepare the site, then serve it until
/// Ctrl-C.
pub fn run(args: MainsailArgs) -> Result<i32, Box<dyn std::error::Error>> {
    run_with(&args, &SystemRunner).map_err(Into::into)
}

/// [`run`] with the program runner given, for tests.
fn run_with(args: &MainsailArgs, runner: &dyn CommandRunner) -> Result<i32, String> {
    let site = prepare_site(args, runner)?;
    serve(args, site)?;
    Ok(0)
}

/// Which layer the site is served from.
#[derive(Clone, Debug, PartialEq, Eq)]
enum SiteChoice {
    /// `--dir`: a site that is already built.
    Given(PathBuf),
    /// The copy compiled into the binary.
    Embedded,
    /// The checkout's `dist/`, built if it needs to be.
    Checkout(PathBuf),
}

impl SiteChoice {
    /// Whether this layer serves a directory (rather than the embedded copy).
    #[cfg(test)]
    fn dir(&self) -> Option<&Path> {
        match self {
            SiteChoice::Given(dir) | SiteChoice::Checkout(dir) => Some(dir),
            SiteChoice::Embedded => None,
        }
    }
}

/// The resolution itself, with every input given, so the tests can drive each
/// layer without a checkout, a built site or the feature.
fn choose_site(
    given: Option<&Path>,
    rebuild: bool,
    embedded_available: bool,
    checkout_dist: &Path,
) -> Result<SiteChoice, String> {
    if let Some(dir) = given {
        if dir.join("index.html").is_file() {
            return Ok(SiteChoice::Given(dir.to_path_buf()));
        }
        return Err(format!(
            "--dir {} has no index.html, so it is not a built site. --dir is served \
             as it is and nothing is built into it: build the checkout with \
             `klipperx mainsail` (no --dir), or point --dir at the dist/ a build \
             made elsewhere.",
            dir.display()
        ));
    }
    if embedded_available && !rebuild {
        return Ok(SiteChoice::Embedded);
    }
    Ok(SiteChoice::Checkout(checkout_dist.to_path_buf()))
}

/// The site as it is served: where its files come from, and a `config.json` held
/// in memory for a site that has nowhere to write one.
#[derive(Clone, Debug)]
struct Site {
    root: SiteRoot,
    /// A `config.json` served instead of a file on disk.
    config_json: Option<Vec<u8>>,
}

/// Where a site's files come from.
#[derive(Clone, Debug)]
enum SiteRoot {
    /// A directory on disk.
    Files(PathBuf),
    /// The table `build.rs` generated: `(path under dist/, bytes)`.
    Embedded(&'static [(&'static str, &'static [u8])]),
}

impl Site {
    /// Whether the site has this file (`relative` is a path under the site).
    fn exists(&self, relative: &str) -> bool {
        if relative == CONFIG_JSON && self.config_json.is_some() {
            return true;
        }
        match &self.root {
            SiteRoot::Files(dir) => dir.join(relative).is_file(),
            SiteRoot::Embedded(files) => files.iter().any(|(name, _)| *name == relative),
        }
    }

    /// The bytes of one file of the site.
    fn read(&self, relative: &str) -> Option<Vec<u8>> {
        if relative == CONFIG_JSON {
            if let Some(bytes) = &self.config_json {
                return Some(bytes.clone());
            }
        }
        match &self.root {
            SiteRoot::Files(dir) => std::fs::read(dir.join(relative)).ok(),
            SiteRoot::Embedded(files) => files
                .iter()
                .find(|(name, _)| *name == relative)
                .map(|(_, bytes)| bytes.to_vec()),
        }
    }

    /// The directory this site is served from, when it is a directory.
    fn dir(&self) -> Option<&Path> {
        match &self.root {
            SiteRoot::Files(dir) => Some(dir),
            SiteRoot::Embedded(_) => None,
        }
    }
}

/// The site file whose content is this machine's Moonraker address.
const CONFIG_JSON: &str = "config.json";

/// The site embedded at build time.
///
/// `build.rs` always writes the table; it is empty unless the
/// `embedded-mainsail` feature was on and a `dist/` existed then.
mod embedded {
    include!(concat!(env!("OUT_DIR"), "/embedded_mainsail.rs"));

    /// The embedded files, `(path under dist/, bytes)`.
    pub fn files() -> &'static [(&'static str, &'static [u8])] {
        FILES
    }

    /// Whether the binary carries a usable site: a site needs an index.
    pub fn available() -> bool {
        files().iter().any(|(name, _)| *name == "index.html")
    }
}

/// Resolve the site, prepare it, and say which layer answered.
fn prepare_site(args: &MainsailArgs, runner: &dyn CommandRunner) -> Result<Site, String> {
    let checkout_dist = mainsail_dist();
    let choice = choose_site(
        args.dir.as_deref(),
        args.rebuild,
        embedded::available(),
        &checkout_dist,
    )?;

    let (root, description) = match &choice {
        SiteChoice::Given(dir) => (
            SiteRoot::Files(dir.clone()),
            format!("--dir {}", dir.display()),
        ),
        SiteChoice::Embedded => (
            SiteRoot::Embedded(embedded::files()),
            format!(
                "embedded in this binary ({} files)",
                embedded::files().len()
            ),
        ),
        SiteChoice::Checkout(dist) => {
            let checkout = mainsail_checkout();
            ensure_site(args, &checkout, dist, runner)?;
            let note = if args.rebuild { ", rebuilt" } else { "" };
            (
                SiteRoot::Files(dist.clone()),
                format!("{} (checkout{note})", dist.display()),
            )
        }
    };

    // One line, always, and first: which path (or which copy) this run serves.
    println!("mainsail: site: {description}");
    with_config_json(
        Site {
            root,
            config_json: None,
        },
        &args.bind,
    )
}

/// What a look at the checkout's build found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SiteState {
    /// `dist/index.html` exists and is newer than the lock file.
    Ready,
    /// Nothing is built: `dist/index.html` is not there.
    Missing,
    /// Built, but from an older dependency tree than the checkout now has.
    Stale,
}

/// Whether the checkout's site needs building, from the two timestamps that
/// decide it.
///
/// The built `index.html` is the marker of a build, the way a lock file is the
/// marker of a dependency tree: `npm ci` installs exactly what
/// `package-lock.json` pins, so a lock file newer than the built index means the
/// site was built from dependencies that are no longer checked out.
fn site_state(dist: &Path, checkout: &Path) -> SiteState {
    let index = dist.join("index.html");
    let Ok(index_time) = std::fs::metadata(&index).and_then(|meta| meta.modified()) else {
        return SiteState::Missing;
    };
    match std::fs::metadata(checkout.join("package-lock.json")).and_then(|meta| meta.modified()) {
        Ok(lock_time) if lock_time > index_time => SiteState::Stale,
        _ => SiteState::Ready,
    }
}

/// Make sure the checkout layer has a build to serve, building one if allowed.
fn ensure_site(
    args: &MainsailArgs,
    checkout: &Path,
    dist: &Path,
    runner: &dyn CommandRunner,
) -> Result<(), String> {
    let state = site_state(dist, checkout);
    if state == SiteState::Ready && !args.rebuild {
        return Ok(());
    }
    if args.no_build {
        return Err(format!(
            "{}; {}",
            site_problem(state, dist, checkout),
            build_hint(checkout)
        ));
    }
    if args.rebuild {
        println!("mainsail: site: rebuilding as asked");
    } else {
        println!("mainsail: site: {}", site_problem(state, dist, checkout));
    }
    build_site(checkout, dist, runner)
}

/// The one sentence saying what is wrong with the checkout's build.
fn site_problem(state: SiteState, dist: &Path, checkout: &Path) -> String {
    match state {
        SiteState::Missing => format!(
            "no built site in {}: {} does not exist",
            dist.display(),
            dist.join("index.html").display()
        ),
        SiteState::Stale => format!(
            "the build in {} is older than {}",
            dist.display(),
            checkout.join("package-lock.json").display()
        ),
        SiteState::Ready => format!("the build in {} is up to date", dist.display()),
    }
}

/// What to type to build the site by hand.
fn build_hint(checkout: &Path) -> String {
    format!(
        "build it with: cd {} && npm ci && npm run build",
        checkout.display()
    )
}

/// Run `npm ci && npm run build` in the checkout.
fn build_site(checkout: &Path, dist: &Path, runner: &dyn CommandRunner) -> Result<(), String> {
    let package = checkout.join("package.json");
    if !package.is_file() {
        return Err(format!(
            "cannot build the site: {} does not exist. third_party/ holds submodule \
             checkouts, and an empty one is not a checkout — initialize it in the main \
             checkout (`git submodule update --init third_party/mainsail`) or point \
             --dir at a build made elsewhere",
            package.display()
        ));
    }
    for invocation in [
        Invocation::new("npm", ["ci"]).in_dir(checkout),
        Invocation::new("npm", ["run", "build"]).in_dir(checkout),
    ] {
        let completed = runner
            .run(&invocation)
            .map_err(|error| command::start_error(&invocation, &error))?;
        if !completed.success() {
            return Err(command::failure_error(&invocation, &completed));
        }
    }
    let index = dist.join("index.html");
    if !index.is_file() {
        return Err(format!(
            "`npm run build` finished but {} does not exist: the site was built \
             somewhere else — check vite.config.ts, or name the built directory with --dir",
            index.display()
        ));
    }
    println!("mainsail: site: built {}", dist.display());
    Ok(())
}

/// Give the served site a `config.json` pointing at this machine's Moonraker.
///
/// A site served from a directory gets the file written into it (only when it is
/// missing, so an operator who pointed theirs elsewhere keeps it). An embedded
/// site has nowhere to write, so an equivalent file is served from memory.
fn with_config_json(mut site: Site, bind: &str) -> Result<Site, String> {
    if site.exists(CONFIG_JSON) {
        return Ok(site);
    }
    match &site.root {
        SiteRoot::Files(dir) => {
            let path = dir.join(CONFIG_JSON);
            std::fs::write(&path, rendered_config_json(bind)?)
                .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
            println!(
                "mainsail: wrote {} (Moonraker at {}:{MOONRAKER_PORT})",
                path.display(),
                browse_host(bind)
            );
        }
        SiteRoot::Embedded(_) => {
            site.config_json = Some(rendered_config_json(bind)?);
            println!(
                "mainsail: serving config.json from memory (Moonraker at {}:{MOONRAKER_PORT})",
                browse_host(bind)
            );
        }
    }
    Ok(site)
}

/// The small configuration Mainsail expects: which Moonraker, and the locale.
fn rendered_config_json(bind: &str) -> Result<Vec<u8>, String> {
    let config = serde_json::json!({
        "defaultLocale": "zh-CN",
        "instances": [{"hostname": browse_host(bind), "port": MOONRAKER_PORT}],
    });
    let mut text = serde_json::to_string_pretty(&config)
        .map_err(|error| format!("cannot encode the site configuration: {error}"))?;
    text.push('\n');
    Ok(text.into_bytes())
}

/// The address a browser on this machine should use for a service bound to
/// `bind`: a wildcard address is not one a browser can open, so it becomes the
/// loopback address of its family.
fn browse_host(bind: &str) -> &str {
    match bind {
        "0.0.0.0" => "127.0.0.1",
        "::" | "[::]" => "::1",
        other => other,
    }
}

/// Serve the site on `bind:port` until Ctrl-C.
fn serve(args: &MainsailArgs, site: Site) -> Result<(), String> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|error| format!("cannot start a runtime: {error}"))?;
    runtime.block_on(async move {
        let listener = bind_listener(&args.bind, args.port).await?;
        let port = listener
            .local_addr()
            .map_err(|error| format!("cannot read the bound address: {error}"))?
            .port();
        let served = match site.dir() {
            Some(dir) => dir.display().to_string(),
            None => "the embedded site".to_string(),
        };
        // Printed after the bind, so `--port 0` names the port actually chosen
        // rather than the zero that was asked for.
        println!(
            "mainsail: serving {served} on http://{}:{port}/ (Ctrl-C to stop)",
            browse_host(&args.bind)
        );
        accept_loop(listener, Arc::new(site), async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await
    })
}

/// Bind the listening socket, saying why when the address cannot be had.
async fn bind_listener(bind: &str, port: u16) -> Result<tokio::net::TcpListener, String> {
    tokio::net::TcpListener::bind((bind, port))
        .await
        .map_err(|error| match error.kind() {
            std::io::ErrorKind::AddrInUse => format!(
                "cannot serve on {bind}:{port}: the port is already in use — stop the \
                 other program or choose another --port"
            ),
            std::io::ErrorKind::AddrNotAvailable => format!(
                "cannot serve on {bind}:{port}: this machine has no such address — \
                 check --bind"
            ),
            _ => format!("cannot serve on {bind}:{port}: {error}"),
        })
}

/// Accept connections until `stop` resolves, one task per connection.
async fn accept_loop(
    listener: tokio::net::TcpListener,
    site: Arc<Site>,
    stop: impl std::future::Future<Output = ()> + Send + 'static,
) -> Result<(), String> {
    tokio::pin!(stop);
    loop {
        tokio::select! {
            accepted = listener.accept() => {
                let (stream, _peer) = accepted.map_err(|error| format!("cannot accept a connection: {error}"))?;
                let site = Arc::clone(&site);
                // One task per connection: a slow page does not hold up the asset
                // that follows it, and nothing is shared but the site.
                tokio::spawn(async move {
                    serve_connection(stream, site).await;
                });
            }
            _ = &mut stop => {
                println!("mainsail: stopping");
                return Ok(());
            }
        }
    }
}

/// Answer one request on one connection, then close it.
async fn serve_connection(mut stream: TcpStream, site: Arc<Site>) {
    let head = match read_request(&mut stream).await {
        Ok(head) => head,
        Err(reply) => {
            let _ = stream.write_all(&reply.into_bytes(false)).await;
            return;
        }
    };
    let Some((method, target)) = request_line(&head) else {
        let reply = Reply::text(400, "malformed request line\n");
        let _ = stream.write_all(&reply.into_bytes(false)).await;
        return;
    };
    let reply = route(&site, method, target);
    let head_only = method.eq_ignore_ascii_case("HEAD");
    let _ = stream.write_all(&reply.into_bytes(head_only)).await;
    let _ = stream.shutdown().await;
}

/// Read up to the empty line that ends the request head.
async fn read_request(stream: &mut TcpStream) -> Result<String, Reply> {
    let mut head = Vec::new();
    let mut chunk = [0u8; 1024];
    loop {
        let read = stream
            .read(&mut chunk)
            .await
            .map_err(|error| Reply::text(400, &format!("cannot read the request: {error}\n")))?;
        if read == 0 {
            break;
        }
        head.extend_from_slice(&chunk[..read]);
        if head.windows(4).any(|window| window == b"\r\n\r\n")
            || head.windows(2).any(|window| window == b"\n\n")
        {
            break;
        }
        if head.len() > MAX_REQUEST_BYTES {
            return Err(Reply::text(400, "request head is too large\n"));
        }
    }
    String::from_utf8(head).map_err(|_| Reply::text(400, "request is not UTF-8\n"))
}

/// The method and target of a request head's first line.
fn request_line(head: &str) -> Option<(&str, &str)> {
    let mut parts = head.lines().next()?.split_whitespace();
    let method = parts.next()?;
    let target = parts.next()?;
    Some((method, target))
}

/// One response: status, content type and body.
#[derive(Debug, PartialEq, Eq)]
struct Reply {
    status: u16,
    content_type: &'static str,
    body: Vec<u8>,
}

impl Reply {
    /// A short plain-text answer, for errors.
    fn text(status: u16, message: &str) -> Reply {
        Reply {
            status,
            content_type: "text/plain; charset=utf-8",
            body: message.as_bytes().to_vec(),
        }
    }

    /// The response as bytes, header block included; `head_only` omits the body
    /// but keeps its length, which is what a `HEAD` must do.
    fn into_bytes(&self, head_only: bool) -> Vec<u8> {
        let head = format!(
            "HTTP/1.1 {} {}\r\nContent-Type: {}\r\nContent-Length: {}\r\n\
             Cache-Control: no-cache\r\nConnection: close\r\n\r\n",
            self.status,
            reason(self.status),
            self.content_type,
            self.body.len()
        );
        let mut bytes = head.into_bytes();
        if !head_only {
            bytes.extend_from_slice(&self.body);
        }
        bytes
    }
}

/// The reason phrase of a status line.
fn reason(status: u16) -> &'static str {
    match status {
        200 => "OK",
        400 => "Bad Request",
        403 => "Forbidden",
        404 => "Not Found",
        405 => "Method Not Allowed",
        500 => "Internal Server Error",
        _ => "Unknown",
    }
}

/// Map one request onto a file of the site and answer it.
fn route(site: &Site, method: &str, target: &str) -> Reply {
    if !(method.eq_ignore_ascii_case("GET") || method.eq_ignore_ascii_case("HEAD")) {
        return Reply::text(405, "this host serves GET and HEAD only\n");
    }

    // Everything before the query and fragment is the path; a request-target is
    // origin-form (`/x/y.html`), so anything else is a proxy request this
    // server is not.
    let raw_path = target.split(['?', '#']).next().unwrap_or_default();
    let decoded = match percent_decode(raw_path) {
        Ok(path) => path,
        Err(reason) => return Reply::text(400, &format!("{reason}\n")),
    };
    if !decoded.starts_with('/') {
        // Only an origin-form request-target (`/x/y`) names a file here.
        return Reply::text(400, &format!("{decoded:?} is not an absolute path\n"));
    }
    let relative = match relative_path(&decoded) {
        Ok(relative) => relative,
        Err(reason) => return Reply::text(403, &format!("{reason}\n")),
    };
    let relative = served_file(site, relative);

    // Second gate, for what the lexical check cannot see: a symlink inside the
    // served directory pointing out of it. Embedded files have no links.
    if let Some(root) = site.dir() {
        if let (Ok(root_real), Ok(file_real)) =
            (root.canonicalize(), root.join(&relative).canonicalize())
        {
            if !file_real.starts_with(&root_real) {
                return Reply::text(403, "the path leaves the served directory\n");
            }
        }
    }

    match site.read(&relative) {
        Some(body) => Reply {
            status: 200,
            content_type: content_type(Path::new(&relative)),
            body,
        },
        None => Reply::text(404, "not found\n"),
    }
}

/// The site file a request path is answered from.
///
/// The empty path is the index; a path that names a directory is that
/// directory's index; anything else unknown is the site index, because a
/// single-page application decides what a URL means in the browser.
fn served_file(site: &Site, relative: String) -> String {
    if relative.is_empty() {
        return "index.html".to_string();
    }
    if site.exists(&relative) {
        return relative;
    }
    let nested = format!("{relative}/index.html");
    if site.exists(&nested) {
        return nested;
    }
    "index.html".to_string()
}

/// The percent-decoded form of a request path.
///
/// Decoding first is what makes the traversal check meaningful: `%2e%2e` is how
/// a request spells `..` while looking harmless to a check on the raw bytes.
fn percent_decode(raw: &str) -> Result<String, String> {
    let bytes = raw.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len());
    let mut index = 0;
    while index < bytes.len() {
        if bytes[index] != b'%' {
            decoded.push(bytes[index]);
            index += 1;
            continue;
        }
        let escape = bytes
            .get(index + 1..index + 3)
            .ok_or_else(|| format!("truncated percent escape in {raw:?}"))?;
        let digits =
            std::str::from_utf8(escape).map_err(|_| format!("bad percent escape in {raw:?}"))?;
        let byte = u8::from_str_radix(digits, 16)
            .map_err(|_| format!("bad percent escape %{digits} in {raw:?}"))?;
        decoded.push(byte);
        index += 3;
    }
    String::from_utf8(decoded).map_err(|_| format!("{raw:?} does not decode to UTF-8"))
}

/// The path under the site a decoded request path names, or why it may not.
///
/// The caller has already checked that `decoded` is an origin-form path (`/…`);
/// what is left to refuse is a segment that climbs out with `..`, which is the
/// one way a path can name something outside the directory — and there is
/// nothing outside the directory to serve. The result uses `/` separators, so it
/// is the same key for a directory and for the embedded table.
fn relative_path(decoded: &str) -> Result<String, String> {
    let rest = decoded.strip_prefix('/').unwrap_or(decoded);
    let mut segments: Vec<&str> = Vec::new();
    for segment in rest.split('/') {
        match segment {
            "" | "." => continue,
            ".." => return Err(format!("{decoded:?} climbs out of the served directory")),
            segment if segment.contains('\0') => {
                return Err(format!("{decoded:?} contains a NUL byte"));
            }
            segment => segments.push(segment),
        }
    }
    Ok(segments.join("/"))
}

/// The `Content-Type` for a file, from its extension.
fn content_type(path: &Path) -> &'static str {
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    match extension.as_str() {
        "html" | "htm" => "text/html; charset=utf-8",
        "js" | "mjs" => "text/javascript; charset=utf-8",
        "css" => "text/css; charset=utf-8",
        "json" | "map" => "application/json",
        "webmanifest" => "application/manifest+json",
        "svg" => "image/svg+xml",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "gif" => "image/gif",
        "webp" => "image/webp",
        "avif" => "image/avif",
        "ico" => "image/x-icon",
        "woff" => "font/woff",
        "woff2" => "font/woff2",
        "ttf" => "font/ttf",
        "otf" => "font/otf",
        "wasm" => "application/wasm",
        "txt" => "text/plain; charset=utf-8",
        "pdf" => "application/pdf",
        _ => "application/octet-stream",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ui::command::Completed;
    use clap::FromArgMatches;
    use std::cell::RefCell;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::time::Duration;

    /// A runner that records what would have run and answers with scripted
    /// output, so a test can watch the build decision without running `npm`.
    struct FakeRunner {
        invocations: RefCell<Vec<Invocation>>,
        status: i32,
        stderr: String,
        side_effect: Option<Box<dyn Fn(&Invocation)>>,
    }

    impl FakeRunner {
        fn ok() -> FakeRunner {
            FakeRunner {
                invocations: RefCell::new(Vec::new()),
                status: 0,
                stderr: String::new(),
                side_effect: None,
            }
        }

        fn failing(stderr: &str) -> FakeRunner {
            FakeRunner {
                status: 1,
                stderr: stderr.to_string(),
                ..FakeRunner::ok()
            }
        }

        fn with_side_effect(mut self, effect: impl Fn(&Invocation) + 'static) -> FakeRunner {
            self.side_effect = Some(Box::new(effect));
            self
        }

        fn calls(&self) -> Vec<String> {
            self.invocations
                .borrow()
                .iter()
                .map(Invocation::command_line)
                .collect()
        }
    }

    impl CommandRunner for FakeRunner {
        fn run(&self, invocation: &Invocation) -> std::io::Result<Completed> {
            self.invocations.borrow_mut().push(invocation.clone());
            if let Some(effect) = &self.side_effect {
                effect(invocation);
            }
            Ok(Completed {
                status: self.status,
                stdout: String::new(),
                stderr: self.stderr.clone(),
            })
        }
    }

    /// A site as `build.rs` would have embedded it, for the layer tests: the
    /// table has the same shape, so the routing code is the code under test.
    static EMBEDDED_SITE: &[(&str, &[u8])] = &[
        ("index.html", b"<html>embedded</html>"),
        ("assets/app.js", b"console.log(2)\n"),
    ];

    /// The same, but carrying a `config.json` of its own.
    static EMBEDDED_SITE_WITH_CONFIG: &[(&str, &[u8])] = &[
        ("index.html", b"<html>embedded</html>"),
        ("config.json", b"{\"mine\":true}"),
    ];

    static SCRATCH: AtomicU32 = AtomicU32::new(0);

    /// A directory of this test's own, named so two runs cannot collide.
    fn scratch(name: &str) -> PathBuf {
        let directory = std::env::temp_dir().join(format!(
            "klipperx-ui-mainsail-{}-{}-{name}",
            std::process::id(),
            SCRATCH.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_dir_all(&directory);
        std::fs::create_dir_all(&directory).expect("a scratch directory");
        directory
    }

    fn args() -> MainsailArgs {
        MainsailArgs {
            port: DEFAULT_PORT,
            bind: DEFAULT_BIND.to_string(),
            dir: None,
            no_build: false,
            rebuild: false,
        }
    }

    /// Parse arguments the way the subcommand does, for the default/`--help`
    /// contract.
    fn parse(arguments: &[&str]) -> Result<MainsailArgs, clap::Error> {
        let command = <MainsailArgs as clap::Args>::augment_args(clap::Command::new("mainsail"));
        let matches = command
            .try_get_matches_from(std::iter::once("mainsail").chain(arguments.iter().copied()))?;
        Ok(MainsailArgs::from_arg_matches(&matches).expect("the arguments parse"))
    }

    /// A checkout-shaped directory: a package plus its lock file.
    fn checkout(directory: &Path) -> (PathBuf, PathBuf) {
        let checkout = directory.join("mainsail");
        std::fs::create_dir_all(&checkout).expect("the checkout");
        std::fs::write(checkout.join("package.json"), "{}").expect("package.json");
        std::fs::write(checkout.join("package-lock.json"), "{}").expect("package-lock.json");
        let dist = checkout.join("dist");
        (checkout, dist)
    }

    /// A built site: `dist/` with an index plus the files the tests ask about.
    fn site_directory(directory: &Path) -> Site {
        let dist = directory.join("dist");
        std::fs::create_dir_all(dist.join("assets")).expect("the built site");
        std::fs::write(dist.join("index.html"), b"<html>mainsail</html>").expect("index.html");
        std::fs::write(dist.join("assets/app.js"), b"console.log(1)\n").expect("app.js");
        std::fs::write(dist.join("assets/app.css"), b"body{}\n").expect("app.css");
        std::fs::write(dist.join("config.json"), b"{}\n").expect("config.json");
        std::fs::write(directory.join("secret"), b"do not serve this\n").expect("the secret");
        Site {
            root: SiteRoot::Files(dist),
            config_json: None,
        }
    }

    #[test]
    fn the_site_layers_are_tried_in_order() {
        let directory = scratch("layers");
        let (checkout, dist) = checkout(&directory);
        let built = directory.join("built");
        std::fs::create_dir_all(&built).expect("a given site");
        std::fs::write(built.join("index.html"), b"<html>given</html>").expect("index.html");

        // 1. `--dir`, whatever the other layers would have answered.
        assert_eq!(
            choose_site(Some(&built), false, true, &dist).expect("a given site wins"),
            SiteChoice::Given(built.clone())
        );
        assert_eq!(
            choose_site(Some(&built), true, true, &dist).expect("--dir still wins"),
            SiteChoice::Given(built.clone())
        );
        // A `--dir` with nothing built in it is a mistake, not a build request.
        let error = choose_site(Some(&dist), false, true, &dist).expect_err("not a built site");
        assert!(error.contains("--dir"), "{error}");
        assert!(error.contains("no index.html"), "{error}");

        // 2. The embedded copy, ahead of the checkout.
        assert_eq!(
            choose_site(None, false, true, &dist).expect("embedded wins"),
            SiteChoice::Embedded
        );
        // …unless a rebuild was asked for, which is about the checkout.
        assert_eq!(
            choose_site(None, true, true, &dist).expect("--rebuild is the checkout"),
            SiteChoice::Checkout(dist.clone())
        );

        // 3. The checkout, when there is nothing embedded.
        assert_eq!(
            choose_site(None, false, false, &dist).expect("the checkout"),
            SiteChoice::Checkout(dist.clone())
        );

        // Only the embedded layer serves no directory of its own.
        assert_eq!(SiteChoice::Embedded.dir(), None);
        assert_eq!(
            SiteChoice::Given(built.clone()).dir(),
            Some(built.as_path())
        );
        assert_eq!(
            SiteChoice::Checkout(dist.clone()).dir(),
            Some(dist.as_path())
        );

        assert_eq!(checkout, directory.join("mainsail"));
        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_arguments_parse_with_their_documented_defaults() {
        let defaults = parse(&[]).expect("every argument has a default");
        assert_eq!(defaults.port, DEFAULT_PORT);
        assert_eq!(defaults.bind, DEFAULT_BIND);
        assert!(defaults.dir.is_none());
        assert!(!defaults.no_build);
        assert!(!defaults.rebuild);

        let given = parse(&["--dir", "/tmp/site", "--no-build", "--port", "9000"]).expect("parsed");
        assert_eq!(given.dir, Some(PathBuf::from("/tmp/site")));
        assert_eq!(given.port, 9000);
        assert!(given.no_build);

        assert!(
            parse(&["--no-build", "--rebuild"]).is_err(),
            "--no-build and --rebuild ask for opposite things"
        );
    }

    #[test]
    fn traversal_is_refused_however_it_is_spelled() {
        let directory = scratch("traversal");
        let site = site_directory(&directory);

        // Plain, dot-segment, and percent-encoded spellings of "climb out".
        for target in [
            "/../secret",
            "/assets/../../secret",
            "/%2e%2e/secret",
            "/assets/%2E%2E/%2e%2e/secret",
            "/./../secret",
        ] {
            assert_eq!(
                route(&site, "GET", target).status,
                403,
                "{target} should be refused"
            );
        }

        // Not an origin-form path at all.
        assert_eq!(route(&site, "GET", "http://elsewhere/secret").status, 400);

        // A NUL byte cannot be part of a file name.
        assert_eq!(route(&site, "GET", "/a%00b").status, 403);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn root_and_unknown_paths_serve_the_index() {
        let directory = scratch("index");
        let site = site_directory(&directory);

        let root = route(&site, "GET", "/");
        assert_eq!(root.status, 200);
        assert_eq!(root.body, b"<html>mainsail</html>");
        assert_eq!(root.content_type, "text/html; charset=utf-8");

        // A client-side route is a page of the application, not a miss.
        let route_like = route(&site, "GET", "/printer/dashboard?x=1");
        assert_eq!(route_like.status, 200);
        assert_eq!(route_like.body, b"<html>mainsail</html>");

        // `HEAD` answers with the same length and no body.
        let head = route(&site, "HEAD", "/");
        assert_eq!(head.status, 200);
        assert_eq!(head.body, b"<html>mainsail</html>");
        let bytes = head.into_bytes(true);
        let text = String::from_utf8_lossy(&bytes);
        assert!(text.contains("Content-Length: 21"), "{text}");
        assert!(text.ends_with("\r\n\r\n"), "HEAD has no body: {text}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn files_are_served_with_their_type() {
        let directory = scratch("types");
        let site = site_directory(&directory);

        let javascript = route(&site, "GET", "/assets/app.js?v=2");
        assert_eq!(javascript.status, 200);
        assert_eq!(javascript.body, b"console.log(1)\n");
        assert_eq!(javascript.content_type, "text/javascript; charset=utf-8");

        assert_eq!(
            route(&site, "GET", "/assets/app.css").content_type,
            "text/css; charset=utf-8"
        );
        assert_eq!(
            route(&site, "GET", "/config.json").content_type,
            "application/json"
        );
        assert_eq!(content_type(Path::new("a.woff2")), "font/woff2");
        assert_eq!(content_type(Path::new("a.svg")), "image/svg+xml");
        assert_eq!(content_type(Path::new("a.wasm")), "application/wasm");
        assert_eq!(content_type(Path::new("a.bin")), "application/octet-stream");
        assert_eq!(content_type(Path::new("a.PNG")), "image/png");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn other_methods_are_refused() {
        let directory = scratch("methods");
        let site = site_directory(&directory);
        assert_eq!(route(&site, "POST", "/").status, 405);
        assert_eq!(route(&site, "DELETE", "/config.json").status, 405);
        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn an_embedded_site_is_served_without_any_checkout() {
        let site = Site {
            root: SiteRoot::Embedded(EMBEDDED_SITE),
            config_json: None,
        };

        let root = route(&site, "GET", "/");
        assert_eq!(root.status, 200);
        assert_eq!(root.body, b"<html>embedded</html>");
        assert_eq!(root.content_type, "text/html; charset=utf-8");

        let javascript = route(&site, "GET", "/assets/app.js");
        assert_eq!(javascript.status, 200);
        assert_eq!(javascript.content_type, "text/javascript; charset=utf-8");

        // The application's routes still fall back to the embedded index …
        assert_eq!(
            route(&site, "GET", "/printer/dashboard").body,
            b"<html>embedded</html>"
        );
        // … and traversal is refused exactly as for a directory.
        assert_eq!(route(&site, "GET", "/../secret").status, 403);
    }

    #[test]
    fn an_embedded_site_gets_a_config_json_from_memory() {
        let site = with_config_json(
            Site {
                root: SiteRoot::Embedded(EMBEDDED_SITE),
                config_json: None,
            },
            "0.0.0.0",
        )
        .expect("the configuration is served from memory");
        assert!(site.config_json.is_some());

        let config = route(&site, "GET", "/config.json");
        assert_eq!(config.status, 200);
        assert_eq!(config.content_type, "application/json");
        let text = String::from_utf8(config.body).expect("UTF-8");
        assert!(text.contains("\"hostname\": \"127.0.0.1\""), "{text}");
        assert!(
            text.contains(&format!("\"port\": {MOONRAKER_PORT}")),
            "{text}"
        );

        // An embedded site that carries its own file keeps it.
        let own = with_config_json(
            Site {
                root: SiteRoot::Embedded(EMBEDDED_SITE_WITH_CONFIG),
                config_json: None,
            },
            "0.0.0.0",
        )
        .expect("its own configuration");
        assert!(own.config_json.is_none());
        assert_eq!(route(&own, "GET", "/config.json").body, b"{\"mine\":true}");
    }

    #[test]
    fn an_empty_directory_is_not_built_when_no_build_was_asked_for() {
        let directory = scratch("nobuild");
        let (checkout, dist) = checkout(&directory);
        let runner = FakeRunner::ok();

        let mut args = args();
        args.no_build = true;
        let error = ensure_site(&args, &checkout, &dist, &runner).expect_err("no build, no site");
        assert!(error.contains("no built site"), "{error}");
        assert!(error.contains("npm ci && npm run build"), "{error}");
        assert!(
            runner.calls().is_empty(),
            "nothing may run: {:?}",
            runner.calls()
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_missing_build_runs_npm_once_and_a_second_run_skips_it() {
        let directory = scratch("build");
        let (checkout, dist) = checkout(&directory);
        let built_dist = dist.clone();
        let runner = FakeRunner::ok().with_side_effect(move |invocation| {
            if invocation.args == ["run", "build"] {
                std::fs::create_dir_all(&built_dist).expect("dist");
                std::fs::write(built_dist.join("index.html"), b"<html>built</html>")
                    .expect("index.html");
            }
        });

        assert_eq!(site_state(&dist, &checkout), SiteState::Missing);
        ensure_site(&args(), &checkout, &dist, &runner).expect("the site is built");
        assert_eq!(runner.calls(), ["npm ci", "npm run build"]);
        assert_eq!(site_state(&dist, &checkout), SiteState::Ready);

        // The second run finds the build and runs nothing at all.
        let second = FakeRunner::ok();
        ensure_site(&args(), &checkout, &dist, &second).expect("the build is reused");
        assert!(
            second.calls().is_empty(),
            "nothing may run: {:?}",
            second.calls()
        );

        // `--rebuild` runs npm even though the build is current.
        let mut forced = args();
        forced.rebuild = true;
        let rebuild = FakeRunner::ok();
        ensure_site(&forced, &checkout, &dist, &rebuild).expect("the build is forced");
        assert_eq!(rebuild.calls(), ["npm ci", "npm run build"]);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_lock_file_newer_than_the_build_makes_it_stale() {
        let directory = scratch("stale");
        let (checkout, dist) = checkout(&directory);
        let built_dist = dist.clone();
        let runner = FakeRunner::ok().with_side_effect(move |invocation| {
            if invocation.args == ["run", "build"] {
                std::fs::create_dir_all(&built_dist).expect("dist");
                std::fs::write(built_dist.join("index.html"), b"<html>built</html>")
                    .expect("index.html");
            }
        });
        ensure_site(&args(), &checkout, &dist, &runner).expect("the site is built");
        assert_eq!(site_state(&dist, &checkout), SiteState::Ready);

        // Touch the lock file after the build: the dependency tree it pins is
        // no longer the one the site was built from.
        std::thread::sleep(Duration::from_millis(20));
        std::fs::write(checkout.join("package-lock.json"), "{\"v\":2}").expect("a new lock file");
        assert_eq!(site_state(&dist, &checkout), SiteState::Stale);

        let rebuild = FakeRunner::ok();
        ensure_site(&args(), &checkout, &dist, &rebuild).expect("a stale build is rebuilt");
        assert_eq!(rebuild.calls(), ["npm ci", "npm run build"]);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_failing_build_reports_what_npm_said() {
        let directory = scratch("npmfails");
        let (checkout, dist) = checkout(&directory);
        let runner = FakeRunner::failing("npm error code EUSAGE\nnpm error missing script: build");

        let error = ensure_site(&args(), &checkout, &dist, &runner).expect_err("npm failed");
        assert!(error.contains("npm ci"), "{error}");
        assert!(error.contains("missing script: build"), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn an_unchecked_out_submodule_says_so() {
        let directory = scratch("nosources");
        let checkout = directory.join("mainsail");
        std::fs::create_dir_all(&checkout).expect("an empty checkout");
        let dist = checkout.join("dist");
        let runner = FakeRunner::ok();
        let error = ensure_site(&args(), &checkout, &dist, &runner).expect_err("nothing to build");
        assert!(error.contains("package.json"), "{error}");
        assert!(error.contains("git submodule update --init"), "{error}");
        assert!(
            runner.calls().is_empty(),
            "nothing may run: {:?}",
            runner.calls()
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_site_config_points_at_this_machines_moonraker() {
        let directory = scratch("configjson");
        let dist = directory.join("dist");
        std::fs::create_dir_all(&dist).expect("dist");

        let site = with_config_json(
            Site {
                root: SiteRoot::Files(dist.clone()),
                config_json: None,
            },
            "0.0.0.0",
        )
        .expect("config.json");
        assert!(site.config_json.is_none());
        let written: serde_json::Value =
            serde_json::from_slice(&std::fs::read(dist.join("config.json")).expect("config.json"))
                .expect("JSON");
        assert_eq!(written["defaultLocale"], "zh-CN");
        assert_eq!(written["instances"][0]["hostname"], "127.0.0.1");
        assert_eq!(written["instances"][0]["port"], MOONRAKER_PORT);

        // An operator's own file is never overwritten.
        std::fs::write(dist.join("config.json"), "{\"mine\":true}").expect("own file");
        with_config_json(
            Site {
                root: SiteRoot::Files(dist.clone()),
                config_json: None,
            },
            "0.0.0.0",
        )
        .expect("config.json");
        let kept = std::fs::read_to_string(dist.join("config.json")).expect("config.json");
        assert_eq!(kept, "{\"mine\":true}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_wildcard_bind_becomes_a_reachable_host() {
        assert_eq!(browse_host("0.0.0.0"), "127.0.0.1");
        assert_eq!(browse_host("::"), "::1");
        assert_eq!(browse_host("192.168.1.5"), "192.168.1.5");
    }

    /// A `GET` over a real socket, answered by the real server.
    async fn get(address: std::net::SocketAddr, target: &str) -> (u16, String) {
        let mut stream = TcpStream::connect(address).await.expect("a connection");
        let request =
            format!("GET {target} HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
        stream
            .write_all(request.as_bytes())
            .await
            .expect("the request");
        let mut response = Vec::new();
        stream.read_to_end(&mut response).await.expect("the reply");
        let text = String::from_utf8_lossy(&response).into_owned();
        let status = text
            .split_whitespace()
            .nth(1)
            .and_then(|code| code.parse().ok())
            .expect("a status code");
        (status, text)
    }

    #[test]
    fn the_site_is_served_over_http() {
        let directory = scratch("http");
        let site = site_directory(&directory);
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("a runtime");

        let bounded = runtime.block_on(async {
            tokio::time::timeout(Duration::from_secs(20), async {
                let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
                    .await
                    .expect("a random port");
                let address = listener.local_addr().expect("the bound address");
                let server = tokio::spawn(accept_loop(
                    listener,
                    Arc::new(site),
                    std::future::pending::<()>(),
                ));

                let (status, text) = get(address, "/").await;
                assert_eq!(status, 200);
                assert!(text.contains("<html>mainsail</html>"), "{text}");
                assert!(text.contains("Content-Length: 21"), "{text}");

                let (status, text) = get(address, "/assets/app.js").await;
                assert_eq!(status, 200);
                assert!(text.contains("text/javascript"), "{text}");

                let (status, _) = get(address, "/../secret").await;
                assert_eq!(status, 403);

                let (status, text) = get(address, "/printer/dashboard").await;
                assert_eq!(status, 200);
                assert!(
                    text.contains("<html>mainsail</html>"),
                    "SPA fallback: {text}"
                );

                server.abort();
            })
            .await
        });
        bounded.expect("the HTTP test is bounded");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }
}
