<#
  串口抓包 / Serial Sniff
  ------------------------------------------------------------------
  不经过浏览器，直接打开 COM 口读原始字节，用来回答一个很具体的问题：
  "这根线上到底有没有字节？"

  判定逻辑（和网页里的计数器一致）：
    0 字节        -> 链路层就没通（埠选错 / 线没接 / 模块没供电 / 波特率差太多）
    有字节但不成帧 -> 波特率不对，或收到的是别的设备的话
    帧数 > 0      -> 波特率正确，链路正常，问题不在线上

  用法：
    .\serial_sniff.ps1                     # 自动列出端口，逐个波特率试
    .\serial_sniff.ps1 -Port COM13
    .\serial_sniff.ps1 -Port COM13 -Baud 115200,9600,57600 -Seconds 8
#>
param(
  [string]$Port = "",
  [int[]]$Baud = @(115200, 9600),
  [int]$Seconds = 6
)

$ErrorActionPreference = "Stop"

# ---------- 端口列表（含设备名，好认出哪个是蓝牙/USB-TTL） ----------
function Get-PortFriendlyNames {
  $map = @{}
  foreach ($root in @('HKLM:\SYSTEM\CurrentControlSet\Enum\USB', 'HKLM:\SYSTEM\CurrentControlSet\Enum\BTHENUM')) {
    if (-not (Test-Path $root)) { continue }
    foreach ($vid in Get-ChildItem $root -ErrorAction SilentlyContinue) {
      foreach ($dev in Get-ChildItem $vid.PSPath -ErrorAction SilentlyContinue) {
        $p = Get-ItemProperty $dev.PSPath -ErrorAction SilentlyContinue
        if ($p.FriendlyName -and ($p.FriendlyName -match '\((COM\d+)\)')) {
          $map[$Matches[1]] = $p.FriendlyName
        }
      }
    }
  }
  return $map
}

$names = Get-PortFriendlyNames
$present = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)
if ($present.Count -eq 0) {
  Write-Host "没有检测到任何串口。检查：USB-TTL 插了吗？HC-05 配对了吗？" -ForegroundColor Red
  exit 1
}

Write-Host ""
Write-Host "===== 当前存在的串口 =====" -ForegroundColor Cyan
foreach ($p in $present) {
  $n = if ($names.ContainsKey($p)) { $names[$p] } else { "(无设备名)" }
  Write-Host ("  {0,-7} {1}" -f $p, $n)
}
Write-Host ""

if (-not $Port) {
  if ($present.Count -eq 1) {
    $Port = $present[0]
    Write-Host "只有一个串口，用它：$Port" -ForegroundColor Yellow
  } else {
    $Port = (Read-Host "要读哪个口（直接输 COM13 这样）").Trim().ToUpper()
  }
}
$Port = $Port.ToUpper()
if ($Port -notmatch '^COM\d+$') { Write-Host "端口名不对：$Port" -ForegroundColor Red; exit 1 }

# 和固件一致的帧格式：10 段整数，逗号分隔
$numLine = '^-?\d+(?:\s*,\s*-?\d+){9}$'
$result = @()

foreach ($b in $Baud) {
  Write-Host ""
  Write-Host "===== $Port @ $b 8N1，读 $Seconds 秒（期间请让小车通电跑起来）=====" -ForegroundColor Cyan
  $sp = $null
  try {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $b, 'None', 8, 'One'
    $sp.ReadTimeout  = 200
    $sp.WriteTimeout = 200
    $sp.DtrEnable = $true      # 有些 CH340 / 蓝牙虚拟口必须拉高 DTR 才收得到数据
    $sp.RtsEnable = $true
    $sp.Open()
    Start-Sleep -Milliseconds 250   # 丢掉打开瞬间的复位噪声

    $buf = New-Object 'System.Collections.Generic.List[byte]'
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $Seconds) {
      $n = $sp.BytesToRead
      if ($n -gt 0) {
        $tmp = New-Object 'byte[]' $n
        $got = $sp.Read($tmp, 0, $n)
        for ($i = 0; $i -lt $got; $i++) { $buf.Add($tmp[$i]) }
      } else {
        Start-Sleep -Milliseconds 30
      }
    }
  }
  catch {
    Write-Host ("打开/读取失败：{0}" -f $_.Exception.Message) -ForegroundColor Red
    Write-Host "  （常见原因：端口被别的程序占着——网页还连着、串口助手还开着）" -ForegroundColor DarkGray
    $result += [pscustomobject]@{ Baud = $b; Bytes = -1; Frames = 0; Note = $_.Exception.Message }
    if ($sp -and $sp.IsOpen) { $sp.Close() }
    continue
  }
  finally {
    if ($sp -and $sp.IsOpen) { try { $sp.Close() } catch {} }
  }

  $bytes = $buf.ToArray()
  $text  = [System.Text.Encoding]::ASCII.GetString($bytes)
  $lines = $text -split "[`r`n]+" | Where-Object { $_.Trim().Length -gt 0 }
  $frames = @($lines | Where-Object { $_.Trim() -match $numLine }).Count

  Write-Host ("收到字节：{0}" -f $bytes.Count)
  Write-Host ("可解析遥测帧：{0}" -f $frames)
  if ($bytes.Count -gt 0) {
    $show = [Math]::Min(96, $bytes.Count)
    $hex = ($bytes[0..($show - 1)] | ForEach-Object { $_.ToString('x2') }) -join ' '
    $asc = -join ($bytes[0..($show - 1)] | ForEach-Object { if ($_ -ge 32 -and $_ -le 126) { [char]$_ } else { '.' } })
    Write-Host "HEX : $hex" -ForegroundColor DarkGray
    Write-Host "ASCII: $asc" -ForegroundColor DarkGray
  }

  if ($bytes.Count -eq 0) {
    Write-Host "判定：这个波特率下 0 字节。" -ForegroundColor Red
    $note = "0 字节"
  } elseif ($frames -gt 0) {
    Write-Host "判定：收到完整遥测帧，波特率和接线都对。" -ForegroundColor Green
    $note = "正常"
  } else {
    Write-Host "判定：有字节但不是遥测帧 —— 多半波特率不对（乱码就是波特率错的典型样子）。" -ForegroundColor Yellow
    $note = "乱码"
  }
  $result += [pscustomobject]@{ Baud = $b; Bytes = $bytes.Count; Frames = $frames; Note = $note }
}

Write-Host ""
Write-Host "===== 汇总 =====" -ForegroundColor Cyan
$result | Format-Table -AutoSize
Write-Host "提示：固件把 UART1 固定设成 115200（ti_msp_dl_config.c 里 UART_1_BAUD_RATE）。" -ForegroundColor DarkGray
Write-Host "      如果只有 9600 能出乱码、115200 是 0 字节，那就是 HC-05/06 出厂默认 9600，" -ForegroundColor DarkGray
Write-Host "      要么用 AT+UART=115200,0,0 改模块，要么把固件波特率改成 9600。" -ForegroundColor DarkGray
