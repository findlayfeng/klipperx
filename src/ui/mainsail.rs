//! `klipperx mainsail` — serve the Mainsail web UI.
//!
//! Mainsail is a Vite single-page application, and what a browser can use is the
//! `dist/` a build of it produces. This subcommand does not build it: a release
//! binary carries the site compiled in (`assets/mainsail/`, embedded by
//! `build.rs`), and a run without one serves a site that is already on disk.
//!
//! Three places the site can come from, in the order `--help` lists them:
//! `--dir <DIR>` (a site built anywhere), the embedded site (the packaged files
//! this binary was built with), and `<repo>/third_party/mainsail/dist` (a
//! checkout someone built by other means — read here, never built). The first
//! that applies is served, and which one it was is printed at startup.
//!
//! The one file Mainsail cannot know — `config.json`, saying which Moonraker to
//! talk to — is written into a site served from disk when it has none. The
//! embedded site's bytes are part of the binary, so a site without that file is
//! reported instead, naming the file to add before the next build.
//!
//! Nothing here runs `npm`, and nothing here reaches the network.
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

use crate::ui::repo_root;

// The packaged site, table and all: `EMBEDDED_MAINSAIL`, `None` when the tree
// this binary was built from carries no `assets/mainsail/`.
include!(concat!(env!("OUT_DIR"), "/mainsail_assets.rs"));

/// The port the site is served on by default.
pub const DEFAULT_PORT: u16 = 8080;

/// The address the site is served on by default: Mainsail is normally opened
/// from another machine on the same network.
pub const DEFAULT_BIND: &str = "0.0.0.0";

/// The port Moonraker listens on, written into the site's `config.json`.
pub const MOONRAKER_PORT: u16 = 7125;

/// The directory a packaged site is put in before a build, relative to the
/// repository root; `build.rs` embeds whatever is under it.
pub const ASSETS_DIR: &str = "assets/mainsail";

/// The largest request head accepted: a request line, some headers, and room to
/// spare. Nothing Mainsail sends is near it.
const MAX_REQUEST_BYTES: usize = 8 * 1024;

/// Where the site comes from, spelled out in `--help`: an operator should be
/// able to tell which one a run will serve, and that none of them builds it.
const SITE_SOURCE_HELP: &str = "\
SITE SOURCE (the first that applies is used):
  1. --dir <DIR>                    a site that is already built; served as it is
  2. the site built into this binary
                                    assets/mainsail/ from the build, printed as
                                    `serving the embedded site (N files)`
  3. <repo>/third_party/mainsail/dist
                                    a build made elsewhere; served read-only

Nothing here runs npm. A release carries the site: put the contents of
Mainsail's dist/ into assets/mainsail/ and build. Which source a run serves is
printed when the service starts.";

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

    /// Serve a site that is already built, instead of the one this binary carries
    ///
    /// The directory must contain `index.html`: `--dir` is served as found, and
    /// nothing is built into it.
    #[arg(long, value_name = "DIR")]
    pub dir: Option<PathBuf>,
}

/// Run `klipperx mainsail`: resolve the site, then serve it until Ctrl-C.
pub fn run(args: MainsailArgs) -> Result<i32, Box<dyn std::error::Error>> {
    let site = prepare_site(&args)?;
    serve(&args, &site).map_err(Into::into)
}

/// The site being served.
///
/// Two shapes, one operation — read one file by its relative path — because the
/// request handling is the same for both; only where the bytes come from
/// differs. A directory site is read on every request, so editing a file shows
/// up on the next reload; an embedded site is fixed at compile time.
#[derive(Clone, Debug)]
enum Site {
    /// A site on disk: `--dir` as given, or a checkout's `dist/`.
    Dir(PathBuf),
    /// The site compiled into this binary, as `build.rs` found it.
    Embedded(&'static [(&'static str, &'static [u8])]),
}

impl Site {
    /// Whether `relative` names a file of this site.
    fn contains(&self, relative: &str) -> bool {
        match self {
            Site::Dir(root) => root.join(relative).is_file(),
            Site::Embedded(files) => embedded_file(files, relative).is_some(),
        }
    }

    /// The bytes of `relative`, or a sentence saying why they cannot be read.
    fn read(&self, relative: &str) -> Result<Vec<u8>, String> {
        match self {
            Site::Dir(root) => std::fs::read(root.join(relative))
                .map_err(|error| format!("cannot read {relative}: {error}")),
            Site::Embedded(files) => embedded_file(files, relative)
                .map(<[u8]>::to_vec)
                .ok_or_else(|| format!("{relative} is not in the embedded site")),
        }
    }

    /// What to print as the site being served.
    fn describe(&self) -> String {
        match self {
            Site::Dir(root) => root.display().to_string(),
            Site::Embedded(files) => format!("the embedded site ({} files)", files.len()),
        }
    }
}

/// One file of an embedded site, by its relative path.
fn embedded_file(
    site: &'static [(&'static str, &'static [u8])],
    relative: &str,
) -> Option<&'static [u8]> {
    site.iter()
        .find(|(path, _)| *path == relative)
        .map(|(_, body)| *body)
}

/// The `dist/` of a checkout under `third_party/`: a site someone built by hand.
fn checkout_dist() -> PathBuf {
    repo_root()
        .join("third_party")
        .join("mainsail")
        .join("dist")
}

/// Resolve the site to serve and leave it with the one file Mainsail cannot
/// know.
fn prepare_site(args: &MainsailArgs) -> Result<Site, String> {
    let site = resolve_site(args, EMBEDDED_MAINSAIL, &checkout_dist())?;
    match &site {
        Site::Dir(dir) => write_config_json(dir, &args.bind)?,
        // The embedded bytes are the binary's own: there is nowhere to write a
        // file that a browser would see, so say what to add and carry on.
        Site::Embedded(files) if embedded_file(files, "config.json").is_none() => println!(
            "mainsail: the embedded site has no config.json, so the page will ask for a \
             server: put one in {ASSETS_DIR}/ and rebuild to have it open on Moonraker at \
             {}:{MOONRAKER_PORT}",
            browse_host(&args.bind)
        ),
        Site::Embedded(_) => {}
    }
    Ok(site)
}

/// The site to serve, chosen from the three sources `--help` lists.
///
/// `embedded` and `checkout` are parameters rather than lookups so a test can
/// drive every branch without a packaged site and without a checkout.
fn resolve_site(
    args: &MainsailArgs,
    embedded: Option<&'static [(&'static str, &'static [u8])]>,
    checkout: &Path,
) -> Result<Site, String> {
    if let Some(dir) = &args.dir {
        if !dir.join("index.html").is_file() {
            return Err(format!(
                "--dir {} has no index.html, so it is not a built site. --dir is served as \
                 it is and nothing is built into it: put the contents of Mainsail's dist/ \
                 there, or drop --dir to serve the site this binary carries.",
                dir.display()
            ));
        }
        return Ok(Site::Dir(dir.clone()));
    }

    // An empty table is a build that embedded nothing, which is what `None`
    // means for the checkout's tree as well. Which source won is printed by
    // [`serve`], once the port it is served on is known.
    if let Some(files) = embedded.filter(|files| !files.is_empty()) {
        return Ok(Site::Embedded(files));
    }

    if checkout.join("index.html").is_file() {
        return Ok(Site::Dir(checkout.to_path_buf()));
    }

    Err(format!(
        "no site to serve: this binary carries none, and {} has no index.html. Put the \
         contents of Mainsail's dist/ into {ASSETS_DIR}/ and rebuild, or name a site that \
         is already built with --dir.",
        checkout.display()
    ))
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

/// Serve `site` on `bind:port` until Ctrl-C.
fn serve(args: &MainsailArgs, site: &Site) -> Result<i32, String> {
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
            site.describe(),
            browse_host(&args.bind)
        );
        accept_loop(listener, Arc::new(site.clone()), async {
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

/// Map one request onto a file of `site` and answer it.
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
    // served directory pointing out of it. An embedded site has no symlinks —
    // its table is the whole of it.
    if let Site::Dir(root) = site {
        if let (Ok(dir_real), Ok(file_real)) =
            (root.canonicalize(), root.join(&relative).canonicalize())
        {
            if !file_real.starts_with(&dir_real) {
                return Reply::text(403, "the path leaves the served directory\n");
            }
        }
    }

    match site.read(&relative) {
        Ok(body) => Reply {
            status: 200,
            content_type: content_type(Path::new(&relative)),
            body,
        },
        Err(reason) => Reply::text(500, &format!("{reason}\n")),
    }
}

/// The file a request path is answered from.
///
/// The empty path is the index; a path that names a directory is that
/// directory's index; anything else unknown is the site index, because a
/// single-page application decides what a URL means in the browser.
fn served_file(site: &Site, relative: String) -> String {
    if relative.is_empty() {
        return "index.html".to_string();
    }
    if site.contains(&relative) {
        return relative;
    }
    let nested = format!("{relative}/index.html");
    if site.contains(&nested) {
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
    use clap::FromArgMatches;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::time::Duration;

    /// A site as `build.rs` would embed it, for tests that have no packaged site
    /// to build with: the JSON config file is deliberately absent, which is the
    /// case the embedded site reports rather than writes.
    const EMBEDDED_FOR_TEST: &[(&str, &[u8])] = &[
        ("index.html", b"<html>embedded</html>"),
        ("assets/app.js", b"console.log(2)\n"),
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

    #[test]
    fn the_arguments_parse_with_their_documented_defaults() {
        let defaults = parse(&[]).expect("every argument has a default");
        assert_eq!(defaults.port, DEFAULT_PORT);
        assert_eq!(defaults.bind, DEFAULT_BIND);
        assert!(defaults.dir.is_none());

        let given = parse(&["--dir", "/tmp/site", "--port", "9000"]).expect("parsed");
        assert_eq!(given.dir, Some(PathBuf::from("/tmp/site")));
        assert_eq!(given.port, 9000);
    }

    #[test]
    fn the_build_switches_are_gone_and_help_names_the_three_sources() {
        // Nothing builds the site any more, so neither switch may come back.
        assert!(parse(&["--no-build"]).is_err(), "--no-build is gone");
        assert!(parse(&["--rebuild"]).is_err(), "--rebuild is gone");

        let mut command =
            <MainsailArgs as clap::Args>::augment_args(clap::Command::new("mainsail"));
        let help = command.render_help().to_string();
        assert!(help.contains("--dir"), "{help}");
        assert!(help.contains("built into this binary"), "{help}");
        assert!(help.contains("third_party/mainsail/dist"), "{help}");
        assert!(!help.contains("no-build"), "{help}");
        assert!(!help.contains("rebuild"), "{help}");
        assert!(!help.contains("npm ci"), "{help}");
    }

    #[test]
    fn a_given_directory_wins_over_the_other_sources() {
        let directory = scratch("given");
        let built = directory.join("built");
        std::fs::create_dir_all(&built).expect("a built site");
        std::fs::write(built.join("index.html"), b"<html>given</html>").expect("index.html");

        // A checkout with a site of its own too, so only precedence can decide.
        let checkout = directory.join("checkout-dist");
        std::fs::create_dir_all(&checkout).expect("the checkout's dist");
        std::fs::write(checkout.join("index.html"), b"<html>checkout</html>").expect("index.html");

        let mut args = args();
        args.dir = Some(built.clone());
        let site = resolve_site(&args, Some(EMBEDDED_FOR_TEST), &checkout).expect("--dir wins");
        assert_eq!(site.describe(), built.display().to_string());
        assert_eq!(
            site.read("index.html").expect("the given index"),
            b"<html>given</html>"
        );

        // The one file the site cannot ship is written into the served directory.
        prepare_site(&args).expect("the site is served");
        assert!(built.join("config.json").is_file());

        // A directory with nothing built in it is a mistake, not a build request.
        args.dir = Some(directory.join("absent"));
        let error =
            resolve_site(&args, Some(EMBEDDED_FOR_TEST), &checkout).expect_err("not a built site");
        assert!(error.contains("no index.html"), "{error}");
        assert!(error.contains("--dir"), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_embedded_site_is_used_when_no_directory_was_given() {
        let directory = scratch("embedded");
        let checkout = directory.join("dist");

        let site = resolve_site(&args(), Some(EMBEDDED_FOR_TEST), &checkout)
            .expect("the embedded site needs nothing on disk");
        assert_eq!(
            site.describe(),
            format!("the embedded site ({} files)", EMBEDDED_FOR_TEST.len())
        );
        assert_eq!(
            site.read("assets/app.js").expect("an embedded file"),
            b"console.log(2)\n"
        );
        assert!(site.contains("index.html"));
        assert!(site.read("nope.js").is_err());

        // A build that embedded nothing falls through to the next source: here
        // there is none, so the operator is told where to put a site.
        let error =
            resolve_site(&args(), Some(&[]), &checkout).expect_err("an empty table is not a site");
        assert!(error.contains(ASSETS_DIR), "{error}");
        assert!(error.contains("--dir"), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_built_checkout_is_the_last_source() {
        let directory = scratch("checkout");
        let dist = built_site(&directory);

        let site = resolve_site(&args(), None, &dist).expect("the checkout's dist is served");
        assert_eq!(site.describe(), dist.display().to_string());
        assert_eq!(
            site.read("index.html").expect("the checkout's index"),
            b"<html>mainsail</html>"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn no_source_at_all_says_what_to_do() {
        let directory = scratch("nothing");
        let absent = directory.join("dist");

        let error = resolve_site(&args(), None, &absent).expect_err("no site to serve");
        assert!(error.contains(ASSETS_DIR), "{error}");
        assert!(error.contains("--dir"), "{error}");
        assert!(error.contains(&absent.display().to_string()), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn traversal_is_refused_however_it_is_spelled() {
        let directory = scratch("traversal");
        let dist = built_site(&directory);

        // Plain, dot-segment, and percent-encoded spellings of "climb out",
        // against both shapes of site: the check is on the path, not on where
        // the bytes come from.
        for site in [Site::Dir(dist.clone()), Site::Embedded(EMBEDDED_FOR_TEST)] {
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
        }

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn root_and_unknown_paths_serve_the_index() {
        let directory = scratch("index");
        let dist = built_site(&directory);
        let site = Site::Dir(dist);

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

        // The embedded site answers the same requests the same way.
        let embedded = Site::Embedded(EMBEDDED_FOR_TEST);
        let root = route(&embedded, "GET", "/");
        assert_eq!(root.status, 200);
        assert_eq!(root.body, b"<html>embedded</html>");
        assert_eq!(
            route(&embedded, "GET", "/assets/app.js").body,
            b"console.log(2)\n"
        );
        assert_eq!(
            route(&embedded, "GET", "/printer/dashboard").body,
            b"<html>embedded</html>",
            "SPA fallback works from the embedded table"
        );
        let head = route(&embedded, "HEAD", "/");
        assert_eq!(head.body, b"<html>embedded</html>");
        let bytes = String::from_utf8_lossy(&head.into_bytes(true)).into_owned();
        assert!(bytes.contains("Content-Length: 21"), "{bytes}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn files_are_served_with_their_type() {
        let directory = scratch("types");
        let dist = built_site(&directory);
        let site = Site::Dir(dist);

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
        assert_eq!(
            route(&Site::Embedded(EMBEDDED_FOR_TEST), "GET", "/assets/app.js").content_type,
            "text/javascript; charset=utf-8"
        );
        assert_eq!(content_type(Path::new("a.woff2")), "font/woff2");
        assert_eq!(content_type(Path::new("a.svg")), "image/svg+xml");
        assert_eq!(content_type(Path::new("a.wasm")), "application/wasm");
        assert_eq!(content_type(Path::new("a.bin")), "application/octet-stream");
        assert_eq!(content_type(Path::new("a.PNG")), "image/png");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_symlink_out_of_the_served_directory_is_refused() {
        let directory = scratch("symlink");
        let dist = built_site(&directory);
        std::os::unix::fs::symlink(directory.join("secret"), dist.join("escape")).expect("symlink");

        // The lexical check passes — the name has no `..` — so this is the
        // canonical-path gate answering.
        let reply = route(&Site::Dir(dist), "GET", "/escape");
        assert_eq!(reply.status, 403);
        let text = String::from_utf8_lossy(&reply.body).into_owned();
        assert!(text.contains("leaves the served directory"), "{text}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn other_methods_are_refused() {
        let directory = scratch("methods");
        let dist = built_site(&directory);
        let site = Site::Dir(dist);
        assert_eq!(route(&site, "POST", "/").status, 405);
        assert_eq!(route(&site, "DELETE", "/config.json").status, 405);
        assert_eq!(
            route(&Site::Embedded(EMBEDDED_FOR_TEST), "PUT", "/").status,
            405
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

    /// A current-thread runtime for one HTTP test.
    fn http_runtime() -> tokio::runtime::Runtime {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("a runtime")
    }

    #[test]
    fn the_site_is_served_over_http() {
        let directory = scratch("http");
        let dist = built_site(&directory);
        let site = Site::Dir(dist.clone());
        let runtime = http_runtime();

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

    #[test]
    fn the_embedded_site_is_served_over_http() {
        let runtime = http_runtime();

        let bounded = runtime.block_on(async {
            tokio::time::timeout(Duration::from_secs(20), async {
                let listener = tokio::net::TcpListener::bind(("127.0.0.1", 0))
                    .await
                    .expect("a random port");
                let address = listener.local_addr().expect("the bound address");
                let server = tokio::spawn(accept_loop(
                    listener,
                    Arc::new(Site::Embedded(EMBEDDED_FOR_TEST)),
                    std::future::pending::<()>(),
                ));

                let (status, text) = get(address, "/").await;
                assert_eq!(status, 200);
                assert!(text.contains("<html>embedded</html>"), "{text}");

                let (status, text) = get(address, "/assets/app.js").await;
                assert_eq!(status, 200);
                assert!(text.contains("console.log(2)"), "{text}");
                assert!(text.contains("text/javascript"), "{text}");

                let (status, text) = get(address, "/printer/dashboard").await;
                assert_eq!(status, 200);
                assert!(
                    text.contains("<html>embedded</html>"),
                    "SPA fallback: {text}"
                );

                let (status, _) = get(address, "/../secret").await;
                assert_eq!(status, 403);

                server.abort();
            })
            .await
        });
        bounded.expect("the HTTP test is bounded");
    }
}
