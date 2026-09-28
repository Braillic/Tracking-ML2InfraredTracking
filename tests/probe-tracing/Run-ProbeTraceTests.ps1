param([string]$SourceRoot = (Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'ML2InfraredTracking'))
$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/ProbeTracing/ProbeTraceBuffer.cs')
$tests = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot 'ProbeTraceTests.cs')
$source = $source + "`n" + ($tests -replace 'using System;','' -replace 'using ProbeTracing;','').Replace('ProbeTraceBuffer','ProbeTracing.ProbeTraceBuffer')

# Exercise the production ASCII exporters with a non-English locale.
$controller = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/ProbeTracing/ProbeSurfaceTrace.cs')
$start = $controller.IndexOf('    private static void WritePointFiles(')
if ($start -lt 0) { throw 'Missing export method' }
$brace = $controller.IndexOf('{', $start)
$depth = 0
for ($i=$brace; $i -lt $controller.Length; $i++) {
    if ($controller[$i] -eq '{') { $depth++ }
    if ($controller[$i] -eq '}') { $depth--; if ($depth -eq 0) { break } }
}
$method = $controller.Substring($start, $i-$start+1).Replace('private static void','public static void')
$source = $source + "`nnamespace ProbeTracing { public static class ProbeExportHarness {" + $method.Replace('CultureInfo.', 'System.Globalization.CultureInfo.').Replace('StreamWriter(', 'System.IO.StreamWriter(').Replace('Path.Combine(', 'System.IO.Path.Combine(').Replace('UTF8Encoding(', 'System.Text.UTF8Encoding(') + '} }'
# Compile actual sampling/export code together and use its TracePoint type.
$types = Add-Type -TypeDefinition $source -PassThru
[ProbeTraceTests]::Run()
$pointType = $types | Where-Object { $_.FullName -eq 'ProbeTracing.TracePoint' }
$exportType = $types | Where-Object { $_.FullName -eq 'ProbeTracing.ProbeExportHarness' }
$point = [Activator]::CreateInstance($pointType)
$point.sessionId='18446744073709551614'; $point.frameId='42'; $point.captureTimeSeconds=1.25
$point.xMetres=.15; $point.yMetres=-.0125; $point.zMetres=.7; $point.confidence=.9; $point.stroke=2
$array = [Array]::CreateInstance($pointType,1); $array.SetValue($point,0)
$directory = Join-Path ([IO.Path]::GetTempPath()) ('probe-export-test-' + [Guid]::NewGuid().ToString('N'))
$previousCulture = [Threading.Thread]::CurrentThread.CurrentCulture
try {
    New-Item -ItemType Directory -Path $directory | Out-Null
    [Threading.Thread]::CurrentThread.CurrentCulture = [Globalization.CultureInfo]::GetCultureInfo('fr-FR')
    [ProbeTracing.ProbeExportHarness]::WritePointFiles([string]$directory,$array)
    $csv = Get-Content -LiteralPath (Join-Path $directory 'trace.csv')
    $pcd = Get-Content -LiteralPath (Join-Path $directory 'trace.pcd')
    if ($csv.Count -ne 2 -or $csv[1].Split(',').Count -ne 8) { throw 'CSV shape/culture failure' }
    if (!$csv[1].StartsWith('18446744073709551614,42,1.25,0.15,-0.0125,0.7,')) { throw 'CSV coordinate/identity failure' }
    if ($pcd -notcontains 'POINTS 1' -or $pcd -notcontains 'WIDTH 1') { throw 'PCD count failure' }
    if ($pcd[-1] -ne '0.15 -0.0125 0.7') { throw 'PCD units/culture failure' }
    if ($pcd -notcontains 'DATA ascii' -or $pcd -notcontains 'FIELDS x y z') { throw 'PCD format failure' }
    '5 production export checks passed'
} finally {
    [Threading.Thread]::CurrentThread.CurrentCulture = $previousCulture
    foreach ($name in @('trace.csv','trace.pcd')) {
        $path = Join-Path $directory $name
        if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path }
    }
    if (Test-Path -LiteralPath $directory) { Remove-Item -LiteralPath $directory }
}
