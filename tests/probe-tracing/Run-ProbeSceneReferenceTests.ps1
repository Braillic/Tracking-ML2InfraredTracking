param([string]$SourceRoot = (Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'ML2InfraredTracking'))
$ErrorActionPreference='Stop'
$source=Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/ProbeTracing/ProbeSurfaceTrace.cs')
function Extract-Method([string]$signature) {
    $start=$source.IndexOf($signature,[StringComparison]::Ordinal)
    if($start -lt 0){throw "Missing $signature"}
    $brace=$source.IndexOf('{',$start);$depth=0
    for($i=$brace;$i -lt $source.Length;$i++) {
        if($source[$i] -eq '{'){$depth++}
        if($source[$i] -eq '}'){$depth--;if($depth -eq 0){return $source.Substring($start,$i-$start+1).Replace('private ','public ')}}
    }
    throw "Unclosed method $signature"
}
$methods=@('private bool TryGetTipOffset(', 'private void UpdateCylinderGeometry(', 'private void AppendVisual(', 'private void ClearVisuals(', 'public void UndoStroke(') | ForEach-Object {Extract-Method $_}
$finite=[regex]::Match($source,'private static bool Finite\(Vector3 v\)=>[^;]+;').Value.Replace('private ','public ')
if(!$finite){throw 'Missing finite validation'}
$harness=Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'ProbeSceneReferenceHarness.cs')
$buffer=Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/ProbeTracing/ProbeTraceBuffer.cs')
# Place the buffer namespace after the harness to keep using directives at the beginning.
$buffer=$buffer.Substring($buffer.IndexOf('namespace ProbeTracing'))
Add-Type -TypeDefinition ($harness.Replace('__METHODS__',(($methods -join "`n")+"`n"+$finite))+"`n"+$buffer)
[ProbeSceneReferenceChecks]::Run()
