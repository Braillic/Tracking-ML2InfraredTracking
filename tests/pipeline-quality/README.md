# Pipeline regression tests

Run `pwsh -NoProfile -File tests/pipeline-quality/Run-PipelineTests.ps1` from the
repository root. The runner uses the repository virtualenv by default; override
`-Python` when needed.

The runner extracts current production sender, conversion, header writer, clock
bridge, percentile window, and pose parser methods. Test-only Unity value types,
clock, and managed math allow deterministic execution outside Unity.

Checks exercise buffer ownership and overwrite behavior, origin/session reset,
rate eligibility, clock mapping failure cases, bounded percentile storage,
pose framing, and 200,016 comparisons against the frozen pre-change conversion.
C# writes real packet fixtures, which Python decodes using its production parser
and compares against FLOAT32/UINT8 mapping expectations. Temporary fixtures are
removed after the test.

These checks supplement a Unity compile; they do not measure Android scheduling,
native clock conversion, sockets on the headset, or physical display timing.
