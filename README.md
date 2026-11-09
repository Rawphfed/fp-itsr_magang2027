# fp-itsr_magang2027

### AI Chats and References:
Custom Teleop
- Terdapat penggunaan AI sebagai helper dalam menggunakan termios dan tty, tetapi histori chat tidak bisa dicantumkan karena berupa overview google. Gen AI digunakan dalam memahami secara ringkas dokumentasi dri kedua komponen tersebut dan syntax yang benar.

Odometry and Global Positioning Reference
- https://claude.ai/share/3350c462-62ee-41b4-b579-72f93822b802
- Penggunaan AI jg berlanjut pada Google Overview untuk suggestion mapping key (const yang ada di atas itu), bug fixing dan perapihan kode,

PID Reference
- https://claude.ai/share/9c54bb33-7e91-4f92-91df-9d029eac2bea
- Penggunaan AI jg berlanjut pada Google Overview untuk bug fixing dan perapihan kode

Doing point e
- https://claude.ai/share/4bddd100-0e24-438f-a1ce-2c84f048b98a
- Penggunaan AI jg berlanjut pada Google Overview untuk bug fixing dan perapihan kode

Image Detector
- https://claude.ai/share/c29b15c7-8012-45b5-8540-ab7349819934
- https://medium.com/srm-mic/color-segmentation-using-opencv-93efa7ac93e2
- Penggunaan AI dilanjutkan di fitur chat google overview; mayoritas koding disini menggunakan AI.

### Penjelasan singkat
- Pada FP ini saya membuat TR (Transport Bot) dengan poin yang dilaksanakan sampai dengan f yakni image detector. 
- Dibuat 2 node yaitu keyboard_control untuk movement dan control melalui keyboard dan obj_detector untuk mendeteksi objek blok biru/merah
- Menggunakan custom mapping keyboard melalui tts dan termios
- Movement local dicapai menggunakan publishing di cmd_vel dengan Twist biasa
- Movement global dicapai menggunakan /Odom dari ROS2
- Foxglove bisa digunakan untuk mengkontrol pergerakan
- PID dibuat dengan menggunakan file python helper
- Saat ini object detector masih belum berhasil
