param(
    [string]$SourceRoot = (Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'ML2InfraredTracking'),
    [string]$Python = '',
    [string]$DesktopRoot = ''
)
$ErrorActionPreference = 'Stop'
function Method([string]$Source, [string]$Signature) {
    $start = $Source.IndexOf($Signature, [StringComparison]::Ordinal)
    if ($start -lt 0) { throw "Missing method: $Signature" }
    $brace = $Source.IndexOf('{', $start)
    $depth = 0
    for ($i = $brace; $i -lt $Source.Length; $i++) {
        if ($Source[$i] -eq '{') { $depth++ }
        if ($Source[$i] -eq '}') {
            $depth--
            if ($depth -eq 0) { return $Source.Substring($start, $i - $start + 1) }
        }
    }
    throw "Unbalanced method: $Signature"
}
$depth = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/Tracking/Implementation/ML2/Runtime/DepthFrameTcpServer.cs')
$raw = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/Tracking/Implementation/ML2/Runtime/ML2DepthRawStream.cs')
$pose = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/Tracking/Implementation/ML2/Runtime/PoseEstimateTcpServer.cs')
$methods = @('public void BeginCaptureEpoch(', 'public bool CanSubmitFrame(', 'internal static double AdvanceSubmitDeadline(',
             'public bool TryGetFrameTimingSnapshot(', 'public struct FrameTimingSnapshot', 'private void RecordLatencySubmit(', 'public void SubmitFrame(', 'internal static void ConvertRawToUInt8Srgb(',
             'private static void WriteHeader(') | ForEach-Object { Method $depth $_ }
$harness = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'PipelineHarness.cs')
$harness = $harness.Replace('__DEPTH__', ($methods -join "`n"))
$harness = $harness.Replace('__CLOCK__', (Method $raw 'internal static double Map('))
$harness = $harness.Replace('__TIMING__', (Method $raw 'public readonly struct CaptureTiming'))
$harness = $harness.Replace('__WINDOW__', (Method $pose 'internal sealed class TimingWindow'))
$harness = $harness.Replace('__PARSE__', (Method $pose 'private static bool TryParsePacket('))
Add-Type -TypeDefinition $harness
$fixtureDirectory = Join-Path ([IO.Path]::GetTempPath()) ('ML2PipelineTests-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $fixtureDirectory | Out-Null
try {
    [PipelineChecks]::Run($fixtureDirectory)
    if (!$Python) { $Python = Join-Path (Split-Path -Parent $SourceRoot) '.venv/Scripts/python.exe' }
    if (!$DesktopRoot) { $DesktopRoot = Join-Path (Split-Path -Parent $SourceRoot) 'desktop' }
    & $Python -B (Join-Path $PSScriptRoot 'verify_wire.py') $DesktopRoot $fixtureDirectory
    if ($LASTEXITCODE -ne 0) { throw 'Cross-language wire checks failed' }
} finally {
    # Delete only the two explicitly named test fixtures and their empty temporary directory.
    foreach ($name in @('float.bin', 'uint8.bin')) {
        $file = Join-Path $fixtureDirectory $name
        if (Test-Path -LiteralPath $file) { Remove-Item -LiteralPath $file }
    }
    Remove-Item -LiteralPath $fixtureDirectory
}
