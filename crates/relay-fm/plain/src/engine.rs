//! Relay File Manager — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_PATH_LEN: usize = 64;

#[derive(Clone, Copy)]
pub struct FilePath {
    pub bytes: [u8; MAX_PATH_LEN],
    pub len: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FmCommand { Copy = 0, Move = 1, Rename = 2, Delete = 3, CreateDir = 4, DeleteDir = 5, Decompress = 6, Concat = 7 }

#[derive(Clone, Copy)]
pub struct FmRequest {
    pub command: FmCommand,
    pub source: FilePath,
    pub dest: FilePath,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum FmValidation { Valid = 0, InvalidPath = 1, PathTooLong = 2, SourceEqDest = 3, InvalidCommand = 4 }

impl FilePath {
    pub const fn empty() -> Self {
        FilePath { bytes: [0u8; MAX_PATH_LEN], len: 0 }
    }

    pub fn from_bytes(src: &[u8]) -> Self {
        let mut path = FilePath::empty();
        let copy_len = if src.len() <= MAX_PATH_LEN { src.len() } else { MAX_PATH_LEN };
        let mut i = 0;
        while i < copy_len {
            path.bytes[i] = src[i];
            i += 1;
        }
        path.len = src.len() as u32;
        path
    }
}

pub fn validate_path(path: &FilePath) -> bool {
    // Reject empty paths
    if path.len == 0 {
        return false;
    }
    // Reject paths exceeding MAX_PATH_LEN
    if path.len as usize > MAX_PATH_LEN {
        return false;
    }
    // Reject paths with null bytes in content
    let len = path.len as usize;
    let mut i: usize = 0;
    while i < len {
        if path.bytes[i] == 0 {
            return false;
        }
        i = i + 1;
    }
    true
}

pub fn paths_equal(a: &FilePath, b: &FilePath) -> bool {
    if a.len != b.len {
        return false;
    }
    let len = a.len as usize;
    if len > MAX_PATH_LEN {
        return false;
    }
    let mut i: usize = 0;
    while i < len {
        if a.bytes[i] != b.bytes[i] {
            return false;
        }
        i = i + 1;
    }
    true
}

pub fn validate_request(req: &FmRequest) -> FmValidation {
    // Validate source path
    if !validate_path(&req.source) {
        if req.source.len as usize > MAX_PATH_LEN {
            return FmValidation::PathTooLong;
        }
        return FmValidation::InvalidPath;
    }
    // Validate dest path (for commands that need it)
    match req.command {
        FmCommand::Delete | FmCommand::DeleteDir => {
            // These commands don't need a dest path
        },
        _ => {
            if !validate_path(&req.dest) {
                if req.dest.len as usize > MAX_PATH_LEN {
                    return FmValidation::PathTooLong;
                }
                return FmValidation::InvalidPath;
            }
        },
    }
    // Check source == dest for Copy/Move/Rename
    match req.command {
        FmCommand::Copy | FmCommand::Move | FmCommand::Rename => {
            if paths_equal(&req.source, &req.dest) {
                return FmValidation::SourceEqDest;
            }
        },
        _ => {},
    }
    FmValidation::Valid
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_path(s: &[u8]) -> FilePath {
        FilePath::from_bytes(s)
    }

    #[test]
    fn test_valid_path() {
        let p = make_path(b"/tmp/file.dat");
        assert!(validate_path(&p));
    }

    #[test]
    fn test_empty_path_rejected() {
        let p = FilePath::empty();
        assert!(!validate_path(&p));
    }

    #[test]
    fn test_path_too_long() {
        let mut p = FilePath::empty();
        p.len = (MAX_PATH_LEN as u32) + 1;
        assert!(!validate_path(&p));
    }

    #[test]
    fn test_null_byte_rejected() {
        let mut p = make_path(b"/tmp/x");
        // Insert a null in the middle
        p.bytes[2] = 0;
        assert!(!validate_path(&p));
    }

    #[test]
    fn test_source_eq_dest_rejected() {
        let src = make_path(b"/data/file.bin");
        let dest = make_path(b"/data/file.bin");
        let req = FmRequest { command: FmCommand::Copy, source: src, dest };
        assert_eq!(validate_request(&req), FmValidation::SourceEqDest);
    }

    #[test]
    fn test_valid_copy_command() {
        let src = make_path(b"/data/a.bin");
        let dest = make_path(b"/data/b.bin");
        let req = FmRequest { command: FmCommand::Copy, source: src, dest };
        assert_eq!(validate_request(&req), FmValidation::Valid);
    }

    #[test]
    fn test_delete_no_dest_needed() {
        let src = make_path(b"/data/old.bin");
        let dest = FilePath::empty(); // empty dest is fine for delete
        let req = FmRequest { command: FmCommand::Delete, source: src, dest };
        assert_eq!(validate_request(&req), FmValidation::Valid);
    }

    #[test]
    fn test_all_validation_variants() {
        // Valid
        let src = make_path(b"/a");
        let dest = make_path(b"/b");
        assert_eq!(validate_request(&FmRequest { command: FmCommand::Move, source: src, dest }), FmValidation::Valid);

        // InvalidPath (empty source)
        assert_eq!(validate_request(&FmRequest { command: FmCommand::Copy, source: FilePath::empty(), dest }), FmValidation::InvalidPath);

        // PathTooLong
        let mut long = FilePath::empty();
        long.len = (MAX_PATH_LEN as u32) + 1;
        assert_eq!(validate_request(&FmRequest { command: FmCommand::Copy, source: long, dest }), FmValidation::PathTooLong);

        // SourceEqDest
        assert_eq!(validate_request(&FmRequest { command: FmCommand::Rename, source: src, dest: src }), FmValidation::SourceEqDest);
    }

    #[test]
    fn test_paths_equal_symmetric() {
        let a = make_path(b"/foo/bar");
        let b = make_path(b"/foo/bar");
        assert!(paths_equal(&a, &b));
        assert!(paths_equal(&b, &a));

        let c = make_path(b"/foo/baz");
        assert!(!paths_equal(&a, &c));
        assert!(!paths_equal(&c, &a));
    }

    #[test]
    fn test_all_commands_valid() {
        let src = make_path(b"/src");
        let dest = make_path(b"/dest");
        let commands = [
            FmCommand::Copy, FmCommand::Move, FmCommand::Rename,
            FmCommand::Delete, FmCommand::CreateDir, FmCommand::DeleteDir,
            FmCommand::Decompress, FmCommand::Concat,
        ];
        for cmd in commands {
            let req = FmRequest { command: cmd, source: src, dest };
            let v = validate_request(&req);
            assert_eq!(v, FmValidation::Valid);
        }
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// FM-P01: empty paths are always rejected by validate_path
    #[kani::proof]
    fn verify_empty_path_rejected() {
        let p = FilePath::empty();
        assert!(!validate_path(&p));
    }

    /// FM-P02: no panics for any symbolic input to validate_request
    #[kani::proof]
    fn verify_no_panic() {
        let cmd_val: u8 = kani::any();
        kani::assume(cmd_val <= 7);
        let command = match cmd_val {
            0 => FmCommand::Copy,
            1 => FmCommand::Move,
            2 => FmCommand::Rename,
            3 => FmCommand::Delete,
            4 => FmCommand::CreateDir,
            5 => FmCommand::DeleteDir,
            6 => FmCommand::Decompress,
            _ => FmCommand::Concat,
        };
        let src_len: u32 = kani::any();
        kani::assume(src_len <= MAX_PATH_LEN as u32 + 2);
        let dest_len: u32 = kani::any();
        kani::assume(dest_len <= MAX_PATH_LEN as u32 + 2);
        let mut source = FilePath::empty();
        source.len = src_len;
        // Fill first byte to avoid null-byte rejection on valid-length paths
        if src_len > 0 && (src_len as usize) <= MAX_PATH_LEN {
            source.bytes[0] = b'/';
        }
        let mut dest = FilePath::empty();
        dest.len = dest_len;
        if dest_len > 0 && (dest_len as usize) <= MAX_PATH_LEN {
            dest.bytes[0] = b'/';
        }
        let req = FmRequest { command, source, dest };
        let _ = validate_request(&req);
    }
}
