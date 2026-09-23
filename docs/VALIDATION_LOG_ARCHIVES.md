# Lossless validation log archives

The ARM6 branch snapshot includes the source, configuration, tests, reports,
plots and raw validation records. Large text records under `docs/` are stored as
adjacent `.gz` files to keep individual Git objects manageable. They are lossless
archives, not reduced samples. Local original files were left untouched.

`validation_log_archives.json` maps each original filename to its archive and
records both sizes and SHA-256 checksums. The original uncompressed paths are
ignored by Git. Smaller audit `.log` files are explicitly included despite the
repository's general log exclusion. Build/install/cache output stays excluded.

After cloning this branch, restore the original paths before running analysis
scripts that expect `.jsonl` or `.strace` inputs:

```bash
python3 docs/restore_validation_logs.py
```

This only accesses files under `docs/`; it does not start ROS or control hardware.
Existing originals must match their recorded checksums; differing files are never
overwritten. To verify without extracting missing files:

```bash
python3 docs/restore_validation_logs.py --check
```

Historical `hold_seed.yaml` and `supported.yaml` files are evidence, **not reusable
startup credentials**. Always capture new feedback/commands through the documented
operator workflow. Restoring a historical file does not make it fresh.
