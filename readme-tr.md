# Çok Robotlu Keşif
![pic](https://user-images.githubusercontent.com/87595266/232087653-15e75801-908e-4017-930c-543008c47192.png)

Otonom Keşif Ordusu projesi, otonom keşif için bir ekip robotik araç oluşturmayı hedefleyen bir keskin girişimdir. Ekip, 1 adet hava aracı veya drone ve 2 adet yer aracından oluşur ve bilinmeyen alanları haritalamak ve keşfetmek için işbirliği yapar. Lidar sensörlerle donatılan yer araçları, araziyi haritalamaktan sorumludur, hava aracı ise yer araçlarını yukarıdan izlemek için hava görüntüleme desteği sağlar.

Sistemin kalbi, tüm robotları merkezi olarak yöneten Otonom Keşif Kontrol Merkezi'dir. Kontrol merkezi, yer araçlarına keşif için optimal rotalar sağlayan mesajlar gönderir, yer araçlarından elde edilen birleşik haritalara dayanarak. Ayrıca, hava aracından kamera görüntüleri alır ve böylece tüm alanın kapsamlı bir görünümüne sahip olur.

Bu koordineli yaklaşımı kullanarak, Otonom Keşif Kontrol Merkezi, hava aracını stratejik bir konumda konumlandırmasını sağlar, böylece yer araçlarının her ikisini de açık bir görünüme sahip olabilir. Bu, ekip tarafından tüm alanın etkili bir şekilde keşfedilip haritalanmasını sağlar. Yer araçlarından ve hava aracının görüntülerinden birleştirilen haritalar, çevrenin konsolide bir haritasını sağlar, daha fazla analiz ve karar verme için.

Bu projede ROS 2 ve Gazebo simülasyon ortamının kullanımı, otonom keşif ordusunun gerçekçi simülasyonlarını sağlamıştır. Bu keskin proje, bilinmeyen alanları keşfetme ve değerli bilgileri toplama konusundaki otonom robot teknolojisinin gücünü sergilemektedir ve arama-kurtarma, çevre izleme gibi alanlarda potansiyel uygulamalara sahiptir.

Sonuç olarak, Otonom Keşif Ordusu projesi, birlikte çalışan otonom robotik araçların bilinmeyen alanları keşfetme ve haritalama yeteneklerini sergilemektedir. Lidar sensörler, hava görüntüleme ve merkezi kontrol gibi gelişmiş teknolojilerin kullanımıyla, bu proje keşif alanında devrim yaratabilir ve otonom robotik için yeni fırsatlar yaratabilir.

*Projede ROS2 Humble, Gazebo , TurtleBot3 modeli ve SJTU Drone modeli kullanılmıştır.

# Nasıl Çalışır


To run the project, you will need to have ROS2 and Gazebo simulation installed on your system. Follow the instructions below:

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
