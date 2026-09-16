param(
    [string]$SourceRoot = (Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'ML2InfraredTracking')
)
$ErrorActionPreference = 'Stop'
function Get-MethodSource([string]$Source, [string]$Signature) {
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
$source = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/ML2IRTracking/PoseEstimateTcpServer.cs')
$methods = @('public void SubmitPose(', 'private void TryApplyPendingPose(',
             'internal static bool IsFreshPose(', 'private static bool IsFinitePose(',
             'private void HideTrackedToolIfTimedOut(') | ForEach-Object { Get-MethodSource $source $_ }
$harness = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'PoseFreshnessHarness.cs')
$harness = $harness.Replace('__PRODUCTION_METHODS__', ($methods -join "`n"))
Add-Type -TypeDefinition $harness
[PoseFreshnessChecks]::Run()
