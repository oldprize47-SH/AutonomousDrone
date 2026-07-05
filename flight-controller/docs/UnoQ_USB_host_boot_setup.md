VIN 전원 시 USB Host 모드 부팅 설정
문제 원인
VIN 전원으로 부팅하면 USB-C에 케이블이 없으므로 PM4125 Type-C 컨트롤러가 role 감지를 못 하고, DWC3 드라이버가 device 모드로 기본 설정됩니다.
해결: systemd 서비스 등록
1단계 - SSH로 UNO Q Linux에 접속

# WiFi 연결된 상태에서
ssh arduino@<UNO_Q_IP주소>
# 또는 USB-C 연결 상태에서 ADB로
adb shell
2단계 - 서비스 파일 생성
sudo nano /etc/systemd/system/usb-host-mode.service



내용:

[Unit]
Description=Force USB-C Host Mode (VIN power workaround)
After=multi-user.target

[Service]
Type=oneshot
ExecStartPre=/bin/sleep 3
ExecStart=/bin/sh -c 'echo host > /sys/kernel/debug/usb/4e00000.usb/mode'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target





- sleep 3 - debugfs 마운트 대기
- 4e00000.usb - QRB2210의 DWC3 USB 컨트롤러 주소
  3단계 - 서비스 등록 및 활성화
  sudo systemctl daemon-reload
  sudo systemctl enable usb-host-mode.service
  sudo systemctl start usb-host-mode.service
  4단계 - 확인



# Host 모드 확인
sudo cat /sys/kernel/debug/usb/4e00000.usb/mode
# → "host" 출력되어야 함
# USB 장치 인식 확인 (RealSense 연결 후)
lsusb
# → Intel Corp. RealSense ... 표시되어야 함
5단계 - 재부팅 후 자동 적용 확인
sudo reboot

# 재부팅 후 다시 SSH 접속
lsusb   # RealSense 표시 확인
참고
- 이 버그는 Arduino가 인지하고 있지만 아직 공식 수정은 안 된 상태 (GitHub arduino/linux-qcom#2)
- 최신 OS 이미지에서는 수정되었을 수 있으므로, 먼저 cat /etc/buildinfo로 확인
- 파일이 존재하면 최신 이미지일 가능성이 있고, 워커라운드 없이 동작할 수도 있습니다
  • 
  Save Memory
  보드 수령 후 VIN 전원 연결하고, 위 서비스만 등록하면 부팅마다 자동으로 USB Host 모드가 활성화됩니다. 그 후 USB-C에 RealSense 꽂으면 바로 인식됩니다.