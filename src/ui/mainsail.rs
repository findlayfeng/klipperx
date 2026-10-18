//! `klipperx mainsail` — serve the Mainsail web UI, building it on first run.
//!
//! Mainsail is a Vite single-page application: the checkout under
//! `third_party/mainsail` is its *source*, and what a browser can use is the
//! `dist/` directory `npm ci && npm run build` produces from it. This subcommand
//! is the missing middle: it builds that directory when it is missing or older
//! than the checkout's `package-lock.json` (unless `--no-build`), serves it over
//! HTTP, and writes the one file Mainsail cannot know — `dist/config.json`,
//! which says which Moonraker to talk to.
//!
//! Two places the site can come from, and the first that applies is used:
//! `--dir <DIR>` for a site that is already built (served as it is), otherwise
//! the checkout's `dist/`. The path actually served is printed at startup.
//!
//! The build runs through [`CommandRunner`] so the tests can drive the decision
//! (build or not) without running `npm`.
//!
//! The HTTP side is deliberately plain: HTTP/1.1, one request per connection,
//! `GET` and `HEAD`, no keep-alive. A path that names a file under the served
//! directory is served with its type, a path that names nothing falls back to
//! `index.html` (the application routes in the browser, so any URL is a page),
//! and a path that tries to climb out with `..` — spelled plainly or
//! percent-encoded — is refused.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use clap::Args;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

use crate::ui::command::{self, CommandRunner, Invocation, SystemRunner};
use crate::ui::repo_root;

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

/// Where the site comes from, spelled out in `--help`: an operator should be
/// able to tell which directory a run will serve.
const SITE_SOURCE_HELP: &str = "\
SITE SOURCE (the first that applies is used):
  1. --dir <DIR>                    a site that is already built; never built
  2. <repo>/third_party/mainsail/dist
                                    built with `npm ci && npm run build` when it is
                                    missing or older than package-lock.json

--no-build serves what exists and never runs npm; --rebuild always runs the
build. The served path is printed when the service starts.";

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

    /// Serve a site that is already built, instead of the checkout's dist/
    ///
    /// The directory must contain `index.html`: `--dir` is served as found and
    /// nothing is built into it.
    #[arg(long, value_name = "DIR")]
    pub dir: Option<PathBuf>,

    /// Serve the checkout's site as it is; never run npm
    ///
    /// With this, a site that is not built is an error naming the command that
    /// would have built it.
    #[arg(long, conflicts_with = "rebuild")]
    pub no_build: bool,

    /// Rebuild the checkout's site even if it looks current
    ///
    /// Runs `npm ci && npm run build` even when `dist/` is newer than the lock
    /// file. It has no effect with `--dir`, which is never built.
    #[arg(long)]
    pub rebuild: bool,
}

/// Run `klipperx mainsail`: prepare the site, then serve it until Ctrl-C.
pub fn run(args: MainsailArgs) -> Result<i32, Box<dyn std::error::Error>> {
    run_with(&args, &SystemRunner).map_err(Into::into)
}

/// [`run`] with the program runner given, for tests.
fn run_with(args: &MainsailArgs, runner: &dyn CommandRunner) -> Result<i32, String> {
    let site = prepare_site(args, runner)?;
    serve(args, &site)
}

/// The Mainsail checkout: the Vite project `npm` runs in.
fn checkout() -> PathBuf {
    repo_root().join("third_party").join("mainsail")
}

/// Resolve the site directory and make sure it is there to serve.
fn prepare_site(args: &MainsailArgs, runner: &dyn CommandRunner) -> Result<PathBuf, String> {
    if let Some(dir) = &args.dir {
        if !dir.join("index.html").is_file() {
            return Err(format!(
                "--dir {} has no index.html, so it is not a built site. --dir is served \
                 as it is and nothing is built into it: build the checkout with \
                 `klipperx mainsail` (no --dir), or point --dir at the dist/ a build \
                 made elsewhere.",
                dir.display()
            ));
        }
        println!("mainsail: site: {} (--dir)", dir.display());
        write_config_json(dir, &args.bind)?;
        return Ok(dir.clone());
    }

    let checkout = checkout();
    let dist = checkout.join("dist");
    ensure_built(args, &checkout, &dist, runner)?;
    println!("mainsail: site: {} (the checkout's dist)", dist.display());
    write_config_json(&dist, &args.bind)?;
    Ok(dist)
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

/// Make sure the checkout's site is built, building it if allowed to.
fn ensure_built(
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
        println!("mainsail: building the checkout's site as asked (--rebuild)");
    } else {
        println!("mainsail: {}", site_problem(state, dist, checkout));
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
    println!("mainsail: built {}", dist.display());
    Ok(())
}

/// Write `config.json` into the served site if it is not there yet.
///
/// The file is the site's whole idea of where its server is, and Mainsail ships
/// none: without it the page opens on its "add a printer" screen. Written only
/// when missing, so an operator who pointed theirs elsewhere keeps it.
fn write_config_json(dist: &Path, bind: &str) -> Result<(), String> {
    let path = dist.join("config.json");
    if path.exists() {
        return Ok(());
    }
    let config = serde_json::json!({
        "defaultLocale": "zh-CN",
        "instances": [{"hostname": browse_host(bind), "port": MOONRAKER_PORT}],
    });
    let mut text = serde_json::to_string_pretty(&config)
        .map_err(|error| format!("cannot encode {}: {error}", path.display()))?;
    text.push('\n');
    std::fs::write(&path, text)
        .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
    println!(
        "mainsail: wrote {} (Moonraker at {}:{MOONRAKER_PORT})",
        path.display(),
        browse_host(bind)
    );
    Ok(())
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

/// Serve `dist` on `bind:port` until Ctrl-C.
fn serve(args: &MainsailArgs, dist: &Path) -> Result<i32, String> {
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
        // Printed after the bind, so `--port 0` names the port actually chosen
        // rather than the zero that was asked for.
        println!(
            "mainsail: serving {} on http://{}:{port}/ (Ctrl-C to stop)",
            dist.display(),
            browse_host(&args.bind)
        );
        accept_loop(listener, Arc::new(dist.to_path_buf()), async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await
    })?;
    Ok(0)
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
    root: Arc<PathBuf>,
    stop: impl std::future::Future<Output = ()> + Send + 'static,
) -> Result<(), String> {
    tokio::pin!(stop);
    loop {
        tokio::select! {
            accepted = listener.accept() => {
                let (stream, _peer) = accepted.map_err(|error| format!("cannot accept a connection: {error}"))?;
                let root = Arc::clone(&root);
                // One task per connection: a slow page does not hold up the asset
                // that follows it, and nothing is shared but the directory.
                tokio::spawn(async move {
                    serve_connection(stream, root).await;
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
async fn serve_connection(mut stream: TcpStream, root: Arc<PathBuf>) {
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
    let reply = route(&root, method, target);
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

/// Map one request onto a file under `root` and answer it.
fn route(root: &Path, method: &str, target: &str) -> Reply {
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
    let relative = served_file(root, relative);

    // Second gate, for what the lexical check cannot see: a symlink inside the
    // served directory pointing out of it.
    if let (Ok(root_real), Ok(file_real)) =
        (root.canonicalize(), root.join(&relative).canonicalize())
    {
        if !file_real.starts_with(&root_real) {
            return Reply::text(403, "the path leaves the served directory\n");
        }
    }

    match std::fs::read(root.join(&relative)) {
        Ok(body) => Reply {
            status: 200,
            content_type: content_type(Path::new(&relative)),
            body,
        },
        Err(error) => Reply::text(500, &format!("cannot read {relative}: {error}\n")),
    }
}

/// The file a request path is answered from.
///
/// The empty path is the index; a path that names a directory is that
/// directory's index; anything else unknown is the site index, because a
/// single-page application decides what a URL means in the browser.
fn served_file(root: &Path, relative: String) -> String {
    if relative.is_empty() {
        return "index.html".to_string();
    }
    if root.join(&relative).is_file() {
        return relative;
    }
    let nested = format!("{relative}/index.html");
    if root.join(&nested).is_file() {
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

/// The path under `root` a decoded request path names, or why it may not.
///
/// The caller has already checked that `decoded` is an origin-form path (`/…`);
/// what is left to refuse is a segment that climbs out with `..`, which is the
/// one way a path can name something outside the directory — and there is
/// nothing outside the directory to serve.
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
    fn built_site(directory: &Path) -> PathBuf {
        let dist = directory.join("dist");
        std::fs::create_dir_all(dist.join("assets")).expect("the built site");
        std::fs::write(dist.join("index.html"), b"<html>mainsail</html>").expect("index.html");
        std::fs::write(dist.join("assets/app.js"), b"console.log(1)\n").expect("app.js");
        std::fs::write(dist.join("assets/app.css"), b"body{}\n").expect("app.css");
        std::fs::write(dist.join("config.json"), b"{}\n").expect("config.json");
        std::fs::write(directory.join("secret"), b"do not serve this\n").expect("the secret");
        dist
    }

    /// A runner that behaves like `npm run build`: it produces `dist/index.html`.
    fn building_runner(dist: &Path) -> FakeRunner {
        let built_dist = dist.to_path_buf();
        FakeRunner::ok().with_side_effect(move |invocation| {
            if invocation.args == ["run", "build"] {
                std::fs::create_dir_all(&built_dist).expect("dist");
                std::fs::write(built_dist.join("index.html"), b"<html>built</html>")
                    .expect("index.html");
            }
        })
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
    fn a_given_directory_is_served_as_it_is() {
        let directory = scratch("given");
        let built = directory.join("built");
        std::fs::create_dir_all(&built).expect("a built site");
        std::fs::write(built.join("index.html"), b"<html>given</html>").expect("index.html");

        let mut args = args();
        args.dir = Some(built.clone());
        let runner = FakeRunner::ok();
        let site = prepare_site(&args, &runner).expect("--dir is served as it is");
        assert_eq!(site, built);
        assert!(
            runner.calls().is_empty(),
            "--dir is never built: {:?}",
            runner.calls()
        );
        // The one file the site cannot ship is written into the served directory.
        assert!(built.join("config.json").is_file());

        // A directory with nothing built in it is a mistake, not a build request.
        args.dir = Some(directory.join("absent"));
        let error = prepare_site(&args, &runner).expect_err("not a built site");
        assert!(error.contains("no index.html"), "{error}");
        assert!(error.contains("--dir"), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn traversal_is_refused_however_it_is_spelled() {
        let directory = scratch("traversal");
        let dist = built_site(&directory);

        // Plain, dot-segment, and percent-encoded spellings of "climb out".
        for target in [
            "/../secret",
            "/assets/../../secret",
            "/%2e%2e/secret",
            "/assets/%2E%2E/%2e%2e/secret",
            "/./../secret",
        ] {
            assert_eq!(
                route(&dist, "GET", target).status,
                403,
                "{target} should be refused"
            );
        }

        // Not an origin-form path at all.
        assert_eq!(route(&dist, "GET", "http://elsewhere/secret").status, 400);

        // A NUL byte cannot be part of a file name.
        assert_eq!(route(&dist, "GET", "/a%00b").status, 403);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn root_and_unknown_paths_serve_the_index() {
        let directory = scratch("index");
        let dist = built_site(&directory);

        let root = route(&dist, "GET", "/");
        assert_eq!(root.status, 200);
        assert_eq!(root.body, b"<html>mainsail</html>");
        assert_eq!(root.content_type, "text/html; charset=utf-8");

        // A client-side route is a page of the application, not a miss.
        let route_like = route(&dist, "GET", "/printer/dashboard?x=1");
        assert_eq!(route_like.status, 200);
        assert_eq!(route_like.body, b"<html>mainsail</html>");

        // `HEAD` answers with the same length and no body.
        let head = route(&dist, "HEAD", "/");
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
        let dist = built_site(&directory);

        let javascript = route(&dist, "GET", "/assets/app.js?v=2");
        assert_eq!(javascript.status, 200);
        assert_eq!(javascript.body, b"console.log(1)\n");
        assert_eq!(javascript.content_type, "text/javascript; charset=utf-8");

        assert_eq!(
            route(&dist, "GET", "/assets/app.css").content_type,
            "text/css; charset=utf-8"
        );
        assert_eq!(
            route(&dist, "GET", "/config.json").content_type,
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
        let dist = built_site(&directory);
        assert_eq!(route(&dist, "POST", "/").status, 405);
        assert_eq!(route(&dist, "DELETE", "/config.json").status, 405);
        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn an_empty_directory_is_not_built_when_no_build_was_asked_for() {
        let directory = scratch("nobuild");
        let (checkout, dist) = checkout(&directory);
        let runner = FakeRunner::ok();

        let mut args = args();
        args.no_build = true;
        let error = ensure_built(&args, &checkout, &dist, &runner).expect_err("no build, no site");
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
        let runner = building_runner(&dist);

        assert_eq!(site_state(&dist, &checkout), SiteState::Missing);
        ensure_built(&args(), &checkout, &dist, &runner).expect("the site is built");
        assert_eq!(runner.calls(), ["npm ci", "npm run build"]);
        assert_eq!(site_state(&dist, &checkout), SiteState::Ready);

        // The second run finds the build and runs nothing at all.
        let second = FakeRunner::ok();
        ensure_built(&args(), &checkout, &dist, &second).expect("the build is reused");
        assert!(
            second.calls().is_empty(),
            "nothing may run: {:?}",
            second.calls()
        );

        // `--rebuild` runs npm even though the build is current.
        let mut forced = args();
        forced.rebuild = true;
        let rebuild = FakeRunner::ok();
        ensure_built(&forced, &checkout, &dist, &rebuild).expect("the build is forced");
        assert_eq!(rebuild.calls(), ["npm ci", "npm run build"]);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_lock_file_newer_than_the_build_makes_it_stale() {
        let directory = scratch("stale");
        let (checkout, dist) = checkout(&directory);
        ensure_built(&args(), &checkout, &dist, &building_runner(&dist))
            .expect("the site is built");
        assert_eq!(site_state(&dist, &checkout), SiteState::Ready);

        // Touch the lock file after the build: the dependency tree it pins is
        // no longer the one the site was built from.
        std::thread::sleep(Duration::from_millis(20));
        std::fs::write(checkout.join("package-lock.json"), "{\"v\":2}").expect("a new lock file");
        assert_eq!(site_state(&dist, &checkout), SiteState::Stale);

        let rebuild = FakeRunner::ok();
        ensure_built(&args(), &checkout, &dist, &rebuild).expect("a stale build is rebuilt");
        assert_eq!(rebuild.calls(), ["npm ci", "npm run build"]);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_failing_build_reports_what_npm_said() {
        let directory = scratch("npmfails");
        let (checkout, dist) = checkout(&directory);
        let runner = FakeRunner::failing("npm error code EUSAGE\nnpm error missing script: build");

        let error = ensure_built(&args(), &checkout, &dist, &runner).expect_err("npm failed");
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
        let error = ensure_built(&args(), &checkout, &dist, &runner).expect_err("nothing to build");
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

        write_config_json(&dist, "0.0.0.0").expect("config.json");
        let written: serde_json::Value =
            serde_json::from_slice(&std::fs::read(dist.join("config.json")).expect("config.json"))
                .expect("JSON");
        assert_eq!(written["defaultLocale"], "zh-CN");
        assert_eq!(written["instances"][0]["hostname"], "127.0.0.1");
        assert_eq!(written["instances"][0]["port"], MOONRAKER_PORT);

        // An operator's own file is never overwritten.
        std::fs::write(dist.join("config.json"), "{\"mine\":true}").expect("own file");
        write_config_json(&dist, "0.0.0.0").expect("config.json");
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
        let dist = built_site(&directory);
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
                    Arc::new(dist.clone()),
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
