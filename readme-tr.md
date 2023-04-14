# Çok Robotlu Keşif
![pic](https://user-images.githubusercontent.com/87595266/232087653-15e75801-908e-4017-930c-543008c47192.png)

[en-readme](https://github.com/abdulkadrtr/multiRobotExploration/blob/main/README.md)

Otonom Keşif Ordusu projesi, otonom keşif için robotik araçlardan oluşan bir ekip oluşturmayı amaçlar. Ekip, bilinmeyen bölgeleri haritalamak ve keşfetmek için işbirliği içinde çalışan 1 hava aracı ve 2 kara aracından oluşur. Lidar sensörleri ile donatılan kara araçları, arazinin haritalanmasından, hava aracı ise kara araçlarının yukarıdan izlenmesi için havadan görüntüleme desteği sağlar.

Sistemin kalbi, tüm robotları merkezi olarak yöneten Otonom Keşif Kontrol Merkezi'dir. Kara araçlarına mesajlar göndererek, kara araçlarından elde edilen haritaları birleştirerek birleştirilmiş haritaya dayalı olarak keşif için en uygun rotaları sağlar. Ayrıca hava aracından kamera görüntüleri alarak tüm alanın kapsamlı bir şekilde havadan görüntülenmesini sağlar.

Otonom Keşif Kontrol Merkezi, bu koordineli yaklaşımı kullanarak, hava aracına her iki kara aracını da net bir şekilde görebileceği stratejik bir konuma konumlandırması için komut verir. Bu, ekibin tüm alanı etkili bir şekilde keşfetmesini ve haritasını çıkarmasını sağlar. Kara araçlarından alınan haritalar ve hava aracından alınan görüntüler çevrenin birleştirilmiş bir haritasını sunarak daha fazla analiz ve karar vermeye olanak tanır.

Bu projede ROS 2 ve Gazebo simülasyon ortamının kullanılması, otonom keşif ordusunun test edilmesi ve geliştirilmesi için gerçekçi simülasyonları kolaylaştırmıştır. Bu son teknoloji proje, arama ve kurtarma, çevresel izleme ve daha fazlası gibi alanlardaki potansiyel uygulamalarla, bilinmeyen arazileri keşfetme ve değerli bilgiler toplama konusunda otonom robotların gücünü sergiliyor.

Sonuç olarak, Otonom Keşif Ordusu projesi, bilinmeyen bölgeleri keşfetmek ve haritasını çıkarmak için birlikte çalışan otonom robotik araçlardan oluşan bir ekibin yeteneklerini göstermektedir. Lidar sensörleri, havadan görüntüleme ve merkezi kontrol gibi teknolojilerin kullanımıyla bu proje, keşif alanında yeni yaklaşımlar  yaratma potansiyeline sahiptir.

*Projede ROS2 Humble, Gazebo , TurtleBot3 modeli ve SJTU Drone modeli kullanılmıştır.

# Nasıl Çalışır


Projeyi çalıştırmak için sisteminizde ROS2 ve Gazebo simülasyonunun kurulu olması gerekmektedir. Aşağıdaki talimatları izleyin:

1 - ROS2 ve Gazebo simülasyonunu sisteminize kurun.

2 - Bir ROS2 çalışma alanı (örneğin, ros2_ws) oluşturun ve içinde bir src dizini oluşturun.

3 - Projeyi aşağıdaki komutu kullanarak src dizinine klonlayın:

`git clone https://github.com/abdulkadrtr/multiRobotExploration.git`

4 - Proje dosyalarını ros2_ws dizininde colcon build komutu ile derleyin:

`cd ros2_ws`

`colcon build`

5 - Projeyi aşağıdaki komutla kaynaklayın. `source install/setup.bash`

6 - Gazebo ortamını ve robotları aşağıdaki komutu kullanarak terminalde başlatın:

`ros2 launch turtlebot3_gazebo multi_robot_launch.py`

7 - Harita birleştirme paketini başlatmak için başka bir terminalde aşağıdaki komutu kullanın:

`ros2 launch merge_map merge_map_launch.py`

Bu, harita birleştirme işlemini başlatacak ve birleştirilmiş haritayı bir RViz2 penceresinde, robotların gerçek 
zamanlı yolları ve dronun canlı görüntü akışıyla birlikte görüntüleyecektir.

8 - Son olarak, üçüncü bir terminalde aşağıdaki komutu kullanarak otonom keşif merkezini başlatın:

`ros2 run multi_robot_exploration control`

Bu adımları takip ederek projeniz artık çalışır durumda olacak ve robotlar ortamı otonom olarak keşfe çıkarak haritaları birleştirip tek bir harita oluşturacaklardır.

# YouTube Proje Tanıtım ve Demo Videosu

https://youtu.be/6FtEvvi4lk4

# Tüm Robotlar Çalışırken Simulasyon Görüntüsü

![Screenshot from 2023-04-14 15-09-45](https://user-images.githubusercontent.com/87595266/232044431-143e2592-d4f9-404b-89fd-243b9af53d68.png)

# ROS 2 Iletişim Ağı

![rosgraph](https://user-images.githubusercontent.com/87595266/232061251-64c3ed55-8297-4057-86f8-11599ae4cfa8.svg)
