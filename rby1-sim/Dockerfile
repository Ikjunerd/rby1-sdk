# 원본 이미지는 root 전제(앱이 /root/exe/app)이고 /root 가 0700 이라
# 비root 실행 시 바이너리에 접근조차 못 한다. 진입 권한만 열어준다.
FROM rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2
RUN chmod 755 /root
