//! Build-time freshness check for the client's built-in G-Code parameter table.
//!
//! `src/gcode_params.rs` is checked in because a published client has no host
//! source to scan (see the generator's docs in `src/bin/gen-gcode-params.rs`).
//! In a checkout, though, the host tree is right there, so this build script
//! re-scans it and fails the build when the checked-in table disagrees — a
//! change to a command's declared parameters is caught while building instead
//! of at test time or, worse, at run time.
//!
//! The scan is the same code the generator and `tests/gcode_params_table.rs`
//! use: `src/gcode_params_scan.rs` is self-contained, so it is included here as
//! a module rather than duplicated.
//!
//! When the host tree is absent — a published crate installed from the
//! registry, which carries only this crate's own files — there is nothing to
//! compare against, so the check is skipped and the build proceeds with the
//! checked-in table as the fallback.

// `#[path]` rather than `include!`: the scanner opens with `//!` module docs,
// which are only valid when the file is compiled as a module of its own.
#[allow(dead_code)]
#[path = "src/gcode_params_scan.rs"]
mod scan;

use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    // Re-run when anything the check reads changes: the checked-in table, the
    // scanner it is compared against, and the host tree it scans.
    println!("cargo::rerun-if-changed=src/gcode_params.rs");
    println!("cargo::rerun-if-changed=src/gcode_params_scan.rs");
    println!("cargo::rerun-if-changed=../../src/core/klippy");

    let host = scan::klippy_dir();
    if !host.is_dir() {
        // No host source next to this crate: a published/installed build. The
        // checked-in table is the fallback, so there is nothing to check.
        return;
    }

    let fresh = match scan::scan() {
        Ok(scan) => scan.table(),
        Err(error) => fail(format!("扫描主机源码树失败：{error}")),
    };
    let checked_in = match read_checked_in() {
        Ok(table) => table,
        Err(error) => fail(error),
    };

    let drifted = describe_drift(&fresh, &checked_in);
    if !drifted.is_empty() {
        fail(format!(
            "内建参数表已过期：{}；请跑 `cargo run -p klippy-client --bin gen-gcode-params` 然后 `cargo fmt --all`",
            drifted.join("；")
        ));
    }
}

/// Report a build failure. `cargo::error` makes Cargo attribute it to this
/// crate and exit non-zero, and `exit(1)` is the belt to that braces.
fn fail(message: String) -> ! {
    println!("cargo::error={message}");
    std::process::exit(1);
}

/// Read the checked-in table from `src/gcode_params.rs`.
fn read_checked_in() -> Result<Vec<(String, Vec<String>)>, String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/gcode_params.rs");
    let source = fs::read_to_string(&path)
        .map_err(|error| format!("无法读取 {}：{error}", path.display()))?;
    parse_table(&source).map_err(|error| format!("无法解析 {}：{error}", path.display()))
}

/// The per-command differences, empty when the two tables agree.
///
/// Compared by name and parameters, not by text: the generator writes one
/// command per line while `cargo fmt` spreads an entry across several, so a
/// textual comparison would report drift that is only formatting.
fn describe_drift(
    fresh: &[(String, Vec<String>)],
    checked_in: &[(String, Vec<String>)],
) -> Vec<String> {
    let fresh: BTreeMap<&str, &[String]> = fresh
        .iter()
        .map(|(name, params)| (name.as_str(), params.as_slice()))
        .collect();
    let checked_in: BTreeMap<&str, &[String]> = checked_in
        .iter()
        .map(|(name, params)| (name.as_str(), params.as_slice()))
        .collect();

    let mut drift = Vec::new();
    for (name, params) in &fresh {
        match checked_in.get(name) {
            None => drift.push(format!("`{name}` 缺失（主机声明，签入表没有）")),
            Some(checked) if *checked != *params => drift.push(format!(
                "`{name}` 参数不同（主机 [{}]，签入表 [{}]）",
                params.join(", "),
                checked.join(", ")
            )),
            Some(_) => {}
        }
    }
    for name in checked_in.keys() {
        if !fresh.contains_key(name) {
            drift.push(format!("`{name}` 多出（签入表有，主机已无）"));
        }
    }
    drift
}

/// The token stream of `pub const BUILTIN: &[(&str, &[&str])] = &[…]`.
///
/// Only what a table entry can contain is named; everything else collapses to
/// [`Tok::Other`], which the parser rejects where it does not belong.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Tok {
    /// A string literal, unescaped.
    Str(String),
    /// One of `(`, `)`, `[`, `]`.
    Brace(char),
    /// A `&`, the start of `&[…]`.
    Amp,
    /// A `,`.
    Comma,
    /// Any other character.
    Other,
}

/// Parse the checked-in table's entries out of the crate's `gcode_params.rs`.
///
/// The grammar is the one the generator writes, one entry per command:
/// `("NAME", &["A", "B"]),`; `cargo fmt` may break it across lines, which the
/// tokenizer does not care about.
fn parse_table(source: &str) -> Result<Vec<(String, Vec<String>)>, String> {
    // The array starts at the `[` of `= &[`; the type's `&[…]` has no `=` before
    // it, so the first `= &[` in the file is the value.
    let open = source
        .find("= &[")
        .ok_or_else(|| "没有找到 `= &[`".to_string())?
        + 3;
    let tokens = tokenize(&source[open..]);

    let mut parser = Parser { tokens, pos: 0 };
    parser.expect(Tok::Brace('['))?;
    let mut table = Vec::new();
    while parser.peek() == Some(&Tok::Brace('(')) {
        table.push(parser.entry()?);
    }
    parser.expect(Tok::Brace(']'))?;
    Ok(table)
}

/// The tokens of `source`, ignoring whitespace and `//` comments.
fn tokenize(source: &str) -> Vec<Tok> {
    let chars: Vec<char> = source.chars().collect();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        match chars[i] {
            c if c.is_whitespace() => i += 1,
            '"' => {
                let (literal, next) = read_string(&chars, i);
                tokens.push(Tok::Str(literal));
                i = next;
            }
            '(' | ')' | '[' | ']' => {
                tokens.push(Tok::Brace(chars[i]));
                i += 1;
            }
            '&' => {
                tokens.push(Tok::Amp);
                i += 1;
            }
            ',' => {
                tokens.push(Tok::Comma);
                i += 1;
            }
            '/' if chars.get(i + 1) == Some(&'/') => {
                while i < chars.len() && chars[i] != '\n' {
                    i += 1;
                }
            }
            _ => {
                tokens.push(Tok::Other);
                i += 1;
            }
        }
    }
    tokens
}

/// The string literal starting at `start` (a `"`), and the index just past its
/// closing quote. Unknown escapes keep the escaped character, which is all a
/// command or parameter name can be.
fn read_string(chars: &[char], start: usize) -> (String, usize) {
    let mut out = String::new();
    let mut i = start + 1;
    while i < chars.len() {
        match chars[i] {
            '\\' => {
                if let Some(&escaped) = chars.get(i + 1) {
                    out.push(escaped);
                    i += 2;
                } else {
                    i += 1;
                }
            }
            '"' => return (out, i + 1),
            c => {
                out.push(c);
                i += 1;
            }
        }
    }
    (out, i)
}

/// A cursor over the entry tokens.
struct Parser {
    tokens: Vec<Tok>,
    pos: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.tokens.get(self.pos)
    }

    fn next(&mut self) -> Result<Tok, String> {
        let token = self
            .tokens
            .get(self.pos)
            .cloned()
            .ok_or_else(|| "条目意外结束".to_string())?;
        self.pos += 1;
        Ok(token)
    }

    fn expect(&mut self, want: Tok) -> Result<(), String> {
        let got = self.next()?;
        if got == want {
            Ok(())
        } else {
            Err(format!("期望 {want:?}，读到 {got:?}"))
        }
    }

    fn string(&mut self) -> Result<String, String> {
        match self.next()? {
            Tok::Str(literal) => Ok(literal),
            other => Err(format!("期望字符串，读到 {other:?}")),
        }
    }

    /// One `("NAME", &[…])` entry, positioned on its opening `(`.
    fn entry(&mut self) -> Result<(String, Vec<String>), String> {
        self.expect(Tok::Brace('('))?;
        let name = self.string()?;
        self.expect(Tok::Comma)?;
        self.expect(Tok::Amp)?;
        self.expect(Tok::Brace('['))?;

        let mut params = Vec::new();
        loop {
            match self.next()? {
                Tok::Brace(']') => break,
                Tok::Str(param) => {
                    params.push(param);
                    match self.peek() {
                        Some(Tok::Comma) => self.pos += 1,
                        Some(Tok::Brace(']')) => {
                            self.pos += 1;
                            break;
                        }
                        other => return Err(format!("期望 `,` 或 `]`，读到 {other:?}")),
                    }
                }
                other => return Err(format!("期望参数名或 `]`，读到 {other:?}")),
            }
        }
        // `cargo fmt` puts a comma after the array in the multi-line form
        // (`&[…],\n),`), so one may sit between the `]` and the `)`.
        if self.peek() == Some(&Tok::Comma) {
            self.pos += 1;
        }
        self.expect(Tok::Brace(')'))?;
        if self.peek() == Some(&Tok::Comma) {
            self.pos += 1;
        }
        Ok((name, params))
    }
}
