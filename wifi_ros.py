import os

wifi_sig = 0
wifi_ssid = "'UCInet Mobile Access'"
nmcli_cmd = f'nmcli d wifi | grep {wifi_ssid}'
nmcli = os.popen(nmcli_cmd).read()
if len(nmcli) > 0:
    h_cmd = f'nmcli d wifi | grep BSSID | grep SIGNAL'
    header = os.popen(h_cmd).read()
    start_i = header.find('SIGNAL')
    wifi_sig = nmcli[start_i:start_i+4]
    wifi_sig = int(wifi_sig.strip())

print(f'{wifi_ssid} signal strength: {wifi_sig}')
