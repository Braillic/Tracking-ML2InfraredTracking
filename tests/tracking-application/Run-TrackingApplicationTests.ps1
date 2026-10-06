param(
    [string]$SourceRoot = (Join-Path $PSScriptRoot '../../ML2InfraredTracking'),
    [string]$UnityReferenceProject = ''
)
$ErrorActionPreference='Stop'
$dotnetRoot=Split-Path (Get-Command dotnet -ErrorAction Stop).Source
$sdk=Get-ChildItem (Join-Path $dotnetRoot 'sdk') -Directory | Where-Object {$_.Name -match '^\d+\.\d+\.\d+$'} | Sort-Object {[version]$_.Name} -Descending | Select-Object -First 1
$compiler=Join-Path $sdk.FullName 'Roslyn/bincore/csc.dll'
$framework=Join-Path $dotnetRoot 'packs/NETStandard.Library.Ref/2.1.0/ref/netstandard2.1/netstandard.dll'
$output=Join-Path ([IO.Path]::GetTempPath()) ('tracking-application-tests-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $output | Out-Null
$common=@('/noconfig','/nostdlib+','/langversion:9.0','/warnaserror+',"/reference:$framework")
$coreDll=Join-Path $output 'Braillic.Tracking.Core.dll'
$appDll=Join-Path $output 'Braillic.Tracking.Application.dll'
$runtimeDll=Join-Path $output 'Braillic.Tracking.Runtime.dll'
$coreFiles=@((Get-ChildItem (Join-Path $SourceRoot 'Assets/Tracking/Core') -Filter '*.cs').FullName)
$appFiles=@((Get-ChildItem (Join-Path $SourceRoot 'Assets/Tracking/Application') -Filter '*.cs').FullName)
& dotnet $compiler @common '/target:library' "/out:$coreDll" @coreFiles
if($LASTEXITCODE){throw 'Core compile failed'}
& dotnet $compiler @common '/target:library' "/reference:$coreDll" "/out:$runtimeDll" @((Get-ChildItem (Join-Path $SourceRoot 'Assets/Tracking/Runtime') -Filter '*.cs').FullName)
if($LASTEXITCODE){throw 'Runtime compile failed'}
$common+="/reference:$runtimeDll"
& dotnet $compiler @common '/target:library' "/reference:$coreDll" "/out:$appDll" @appFiles
if($LASTEXITCODE){throw 'Application compile failed'}
$testDll=Join-Path $output 'TrackingApplicationTests.dll'
& dotnet $compiler @common '/target:exe' "/reference:$coreDll" "/reference:$appDll" "/out:$testDll" (Join-Path $PSScriptRoot 'TrackingApplicationTests.cs')
if($LASTEXITCODE){throw 'Test compile failed'}
$major=([version]$sdk.Name).Major
$config=@{runtimeOptions=@{tfm="net$major.0";framework=@{name='Microsoft.NETCore.App';version="$major.0.0"}}}|ConvertTo-Json -Depth 5
[IO.File]::WriteAllText((Join-Path $output 'TrackingApplicationTests.runtimeconfig.json'),$config)
& dotnet $testDll
if($LASTEXITCODE){throw 'Application behavior checks failed'}

$backendDll=Join-Path $output 'BackendTests.dll'
& dotnet $compiler @common '/target:exe' "/reference:$coreDll" "/reference:$appDll" "/out:$backendDll" (Join-Path $PSScriptRoot 'BackendTests.cs') (Join-Path $SourceRoot 'Assets/Tracking/Implementation/ML2/Compatibility/Ml2DesktopTrackingBackend.cs') (Join-Path $SourceRoot 'Assets/Tracking/Unity/TrackingCoordinates.cs')
if($LASTEXITCODE){throw 'Backend test compile failed'}
[IO.File]::WriteAllText((Join-Path $output 'BackendTests.runtimeconfig.json'),$config)
& dotnet $backendDll
if($LASTEXITCODE){throw 'Backend behavior checks failed'}

$unifiedDll=Join-Path $output 'UnifiedTrackingTests.dll'
& dotnet $compiler @common '/target:exe' "/reference:$coreDll" "/reference:$appDll" "/out:$unifiedDll" (Join-Path $PSScriptRoot 'UnifiedTrackingTests.cs')
if($LASTEXITCODE){throw 'Unified tracking test compile failed'}
[IO.File]::WriteAllText((Join-Path $output 'UnifiedTrackingTests.runtimeconfig.json'),$config)
& dotnet $unifiedDll
if($LASTEXITCODE){throw 'Unified tracking behavior checks failed'}

$providerDll=Join-Path $output 'Ml2ProviderTests.dll'
& dotnet $compiler @common '/target:exe' '/main:Ml2ProviderTests' "/reference:$coreDll" "/reference:$appDll" "/out:$providerDll" (Join-Path $PSScriptRoot 'BackendTests.cs') (Join-Path $PSScriptRoot 'Ml2ProviderTests.cs') (Join-Path $SourceRoot 'Assets/Tracking/Implementation/ML2/Compatibility/Ml2DesktopTrackingBackend.cs') (Join-Path $SourceRoot 'Assets/Tracking/Implementation/ML2/Ml2TrackingProvider.cs') (Join-Path $SourceRoot 'Assets/Tracking/Unity/TrackingCoordinates.cs')
if($LASTEXITCODE){throw 'ML2 provider test compile failed'}
[IO.File]::WriteAllText((Join-Path $output 'Ml2ProviderTests.runtimeconfig.json'),$config)
& dotnet $providerDll
if($LASTEXITCODE){throw 'ML2 provider behavior checks failed'}

if($UnityReferenceProject) {
    # Recompile the actual Unity runtime sources against its installed SDK references.
    # Redirect all compiler outputs; never overwrite Library or require an editor launch.
    $rsp=Get-ChildItem (Join-Path $UnityReferenceProject 'Library/Bee/artifacts') -Recurse -Filter 'Assembly-CSharp.rsp' | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if(!$rsp){throw 'Compile the reference Unity project once to create its SDK response file'}
    $lines=Get-Content -LiteralPath $rsp.FullName
    $unityDll=Join-Path $output 'Braillic.Tracking.Unity.dll'
    # Compile the portable Unity integration without any Magic Leap SDK reference.
    $unityRefs=@($lines | Where-Object {$_ -match '^[-/]r(eference)?:.*UnityEngine\.(CoreModule|PhysicsModule)\.dll'})
    & dotnet $compiler @common '/target:library' '/nowarn:0649' "/reference:$coreDll" "/reference:$appDll" @unityRefs "/out:$unityDll" @((Get-ChildItem (Join-Path $SourceRoot 'Assets/Tracking/Unity') -Filter '*.cs').FullName)
    if($LASTEXITCODE){throw 'Portable Unity assembly compile failed'}
    $mapped=[Collections.Generic.List[string]]::new()
    $sources=[Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
    foreach($line in $lines) {
        if($line -match '^[-/](out|refout|analyzer|additionalfile):'){continue}
        if($line -match '^[-/]r(eference)?:.*Braillic\.Tracking\.(Core|Application|Runtime|Unity)\.dll'){continue}
        if($line -match '^"(Assets/.*\.cs)"$') {
            $rel=$Matches[1]
            if($rel -match '^Assets/Tracking/(Core|Runtime|Application|Unity)/' -and $rel -notlike '*Ml2DesktopTrackingBackend.cs'){continue}
            if($rel -like 'Assets/ML2IRTracking/*') {
                $moved='Assets/Tracking/Implementation/ML2/Runtime/'+[IO.Path]::GetFileName($rel)
                if(Test-Path -LiteralPath (Join-Path $SourceRoot $moved)){$rel=$moved}
            }
            if($rel -eq 'Assets/Tracking/Unity/Ml2DesktopTrackingBackend.cs'){$rel='Assets/Tracking/Implementation/ML2/Compatibility/Ml2DesktopTrackingBackend.cs'}
            $candidate=Join-Path $SourceRoot $rel
            if(!(Test-Path -LiteralPath $candidate)){$candidate=Join-Path $UnityReferenceProject $rel}
            $full=[IO.Path]::GetFullPath($candidate)
            if($sources.Add($full)){$mapped.Add('"'+$full+'"')}
        } else {$mapped.Add($line)}
    }
    foreach($file in Get-ChildItem (Join-Path $SourceRoot 'Assets/Tracking/Implementation') -Filter '*.cs' -Recurse) {
        if($sources.Add($file.FullName)){$mapped.Add('"'+$file.FullName+'"')}
    }
    # Older Unity response files may predate tracing's addition to the demo.
    foreach($file in Get-ChildItem (Join-Path $SourceRoot 'Assets/ProbeTracing') -Filter '*.cs') {
        if($sources.Add($file.FullName)){$mapped.Add('"'+$file.FullName+'"')}
    }
    $mapped.Add('-reference:"'+$coreDll+'"');$mapped.Add('-reference:"'+$appDll+'"')
    $mapped.Add('-reference:"'+$runtimeDll+'"')
    $mapped.Add('-reference:"'+$unityDll+'"')
    $mapped.Add('-out:"'+(Join-Path $output 'TrackingApplication.Unity.dll')+'"')
    $buildRsp=Join-Path $output 'Unity.rsp'
    [IO.File]::WriteAllLines($buildRsp,$mapped)
    Push-Location $UnityReferenceProject
    try { & dotnet $compiler ('@'+$buildRsp); if($LASTEXITCODE){throw 'Unity runtime source compilation failed'} }
    finally {Pop-Location}
    'PASS: Unity runtime source compilation (existing SDK references; not a player build)'
}
Write-Output "Build artifacts: $output"
