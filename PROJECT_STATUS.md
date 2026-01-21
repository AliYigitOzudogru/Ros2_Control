# AERO Rover - ROS 2 Control Projesi - Durum Raporu

## 📋 Proje Özeti
4 tekerlekli rover + 4-DOF robotik kol sistemi. ROS 2 Humble + Ignition Gazebo simülasyonu + PS4 DualShock kontrolü.

## 🎮 Kontrol Sistemi
- **R2 Trigger**: İleri hareket (0-3 m/s)
- **Sol Analog Stick**: Direksiyon kontrolü
- **Sağ Analog Stick**: Robotik kol kontrolü
- **L1/R1**: Gripper aç/kapat

## 🚀 Hızlı Başlangıç

### Simülasyonu Başlat
```bash
cd ~/Desktop/Ros2_Control
source install/setup.bash
ros2 launch aero_bringup aero_ignition.launch.py
```

### Build
```bash
colcon build --packages-select aero_description aero_bringup --symlink-install
```

## ✅ Çözülen Problemler (Bu Oturumda)

### 1. PS4 Controller R2 Trigger Hatası ✅
- **Sorun**: R2 trigger yanlış eksen kullanıyordu (axes[4])
- **Çözüm**: axes[5] olarak düzeltildi
- **Formül**: `gas = (1.0 - r2_value) / 2.0`
- **Dosya**: `src/aero_arm_control/aero_arm_control/ps4_rover_controller.py`

### 2. gz_ros2_control Race Condition ✅
- **Sorun**: Plugin `/robot_description` topic'inden URDF alamıyordu
- **Hata**: "Failed to get /robot_description service"
- **Kök Neden**: gz_ros2_control 0.7.17'de bilinen race condition bug
- **Çözüm**: Direct URDF string injection ile launch file'dan doğrudan XML gönderimi
- **Dosya**: `src/aero_bringup/launch/aero_ignition.launch.py`
```python
robot_desc_xml = xacro.process_file(urdf_path, ...).toxml()
spawn_entity = Node(..., arguments=['-string', robot_desc_xml])
```

### 3. Robot Hareket Etmeme - Missing ros2_control Tags ✅
- **Sorun**: Controllers yükleniyor ama tekerlek interface'leri [unclaimed]
- **Kök Neden**: URDF'de `<ros2_control>` ve joint tanımları eksikti
- **Çözüm**: `aero.ignition.xacro`'ya eksiksiz ros2_control yapısı eklendi
  - 4 tekerlek velocity command interfaces
  - 4 kol position command interfaces
  - IMU sensor interfaces
- **Dosya**: `src/aero_description/urdf/aero.ignition.xacro`

### 4. Yanlış Tekerlek Parametreleri ✅
- **Başlangıç Değerleri**: 
  - `wheel_radius: 0.2 m` ❌
  - `wheel_separation: 1.1 m` ❌
- **Analiz**: URDF `base_*.xacro` joint origin'lerinden hesaplandı
  ```
  front_right: xyz="0.407 -0.452 -0.326"
  back_left:   xyz="-0.498 0.451 -0.326"
  ```
  - Z origin: -0.326 m → **wheel_radius: 0.326 m**
  - Y farkı: 0.451 - (-0.452) = 0.903 → **wheel_separation: 0.9 m**
- **Dosya**: `src/aero_bringup/config/controllers.yaml`

### 5. Hız Limitleri Düşük ⚠️
- **Önceki**: linear 5 m/s, angular 3 rad/s
- **Güncelleme**: linear 10 m/s, angular 6 rad/s
- **Acceleration**: 5.0 m/s², 4.0 rad/s²
- **Not**: Test edilmedi, robot henüz hareket etmiyor

## 📊 Mevcu Durum (21 Ocak 2026, Gece)

### ✅ Çalışan Sistemler
- ✅ Ignition Gazebo başlatılıyor
- ✅ Robot spawn ediliyor (direct URDF injection ile)
- ✅ gz_ros2_control plugin yükleniyor
- ✅ Hardware interfaces tanımlanıyor (4 wheel + 4 arm + IMU)
- ✅ PS4 controller bağlanıyor ve komut gönderiyor
- ✅ R2 trigger doğru formül ile 0-3 m/s hız komutu oluşturuyor

### ❌ Çalışmayan / Sorunlu
- ❌ Controllers "already loaded" hatası
- ❌ Spawner'lar başarısız oluyor
- ❌ Robot fiziksel olarak hareket etmiyor
- ⚠️ Controller auto-load ile manuel spawn çakışması

### 🔧 Son Durum
```
[ERROR] gz_ros2_control found an empty parameters file. Failed to initialize.
[spawner] Controller already loaded, skipping load_controller
[spawner] Failed to configure controller
```

**Problem**: 
- Plugin'den parameters tag'i kaldırıldı → boş config hatası
- Launch'tan spawner'lar çalışıyor → "already loaded" hatası
- İki sistem çakışıyor

**Son Değişiklik**: 
- `aero.ignition.xacro`'ya parameters path geri eklendi:
  ```xml
  <parameters>$(find aero_bringup)/config/controllers.yaml</parameters>
  ```

## 📁 Kritik Dosyalar

### Launch Files
- `src/aero_bringup/launch/aero_ignition.launch.py` - Ana launch file (URDF string injection)

### Configuration
- `src/aero_bringup/config/controllers.yaml` - Controller parametreleri
  - diff_drive_controller: wheel_radius, wheel_separation, velocity limits
  - arm_controller: 4-DOF trajectory control
  - joint_state_broadcaster

### URDF/Xacro
- `src/aero_description/urdf/aero.ignition.xacro` - ros2_control tanımları
  - `<ros2_control>` tag with GazeboSimSystem
  - 4 wheel joint velocity interfaces
  - 4 arm joint position interfaces
  - IMU sensor interfaces
- `src/aero_description/urdf/aero_base.xacro` - Tekerlek joint origin'leri

### Control Nodes
- `src/aero_arm_control/aero_arm_control/ps4_rover_controller.py` - PS4 controller node

## 🔄 Sonraki Adımlar (Akşam Devam)

### Öncelik 1: Controller Loading Çakışmasını Çöz
**Seçenek A** (Önerilen):
1. Launch file'dan spawner node'larını kaldır
2. Plugin'in auto-load yapmasına izin ver
3. Test et: `ros2 control list_controllers`

**Seçenek B**:
1. Plugin'den parameters tag'ini kaldır
2. Sadece launch spawner'larını kullan
3. Kontrol et: interface claiming

### Öncelik 2: Hardware Interface Claiming
```bash
ros2 control list_hardware_interfaces
# Beklenen: [available] [claimed]
# Şu an: [available] [unclaimed] veya hiç görünmüyor
```

### Öncelik 3: Fiziksel Hareket Testi
1. R2 trigger test → /cmd_vel komutu
2. Odometry kontrol → /odom topic
3. Joint states kontrol → /joint_states topic
4. Gazebo'da görsel hareket

### Öncelik 4: Hız Limiti Optimizasyonu
- 10 m/s çok fazla olabilir (36 km/h)
- Real-world test sonrası ayarlama

## 🐛 Bilinen Sorunlar

### 1. Controller Spawner Çakışması
```
Controller already loaded, skipping load_controller
Failed to configure controller
```
**Neden**: Plugin auto-load + Launch manual spawn

### 2. Empty Parameters File Error
```
gz_ros2_control found an empty parameters file
```
**Neden**: Plugin'de parameters tag eksik veya path yanlış

### 3. Joint Claiming Sorunu
Wheel velocity interfaces [unclaimed] kalıyor

## 📝 Teknik Notlar

### Ignition vs Gazebo Classic
- ✅ Plugin: `gz_ros2_control` 
- ❌ YANLIŞ: `ign_ros2_control`, `gazebo_ros2_control`
- ROS 2 Humble için: `gz_ros2_control 0.7.17`
- Race condition bug var → direct string injection zorunlu

### ros2_control Yapısı
```xml
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    
    <joint name="base_front_right_wheel_joint">
        <command_interface name="velocity">
            <param name="min">-10</param>
            <param name="max">10</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    <!-- 3 more wheel joints -->
    
    <joint name="arm_joint0">
        <command_interface name="position">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    <!-- 3 more arm joints -->
    
    <sensor name="imu_sensor">
        <state_interface name="orientation.x"/>
        <!-- ... diğer IMU interfaces -->
    </sensor>
</ros2_control>
```

### Wheel Parameters (Doğru Değerler)
```yaml
wheel_radius: 0.326     # meters (URDF Z origin'den)
wheel_separation: 0.9   # meters (Y ekseni farkı)
```

### Velocity Limits (Güncel - Test Edilmedi)
```yaml
linear.x.max_velocity: 10.0      # m/s (36 km/h)
angular.z.max_velocity: 6.0      # rad/s
linear.x.max_acceleration: 5.0   # m/s²
angular.z.max_acceleration: 4.0  # rad/s²
```

## 🔍 Debug Komutları

### Controller Durumu
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_controller_types
```

### Topic Monitoring
```bash
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped
ros2 topic echo /diff_drive_controller/odom --field twist.twist.linear
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

### Parametre Kontrolü
```bash
ros2 param get /diff_drive_controller wheel_radius
ros2 param get /diff_drive_controller wheel_separation
```

## 📚 Referanslar
- [gz_ros2_control GitHub](https://github.com/ros-controls/gz_ros2_control)
- [gz_ros2_control Known Issues](https://github.com/ros-controls/gz_ros2_control/issues)
- ROS 2 Humble ros2_control documentation
- diff_drive_controller documentation

---

**Son Güncelleme**: 21 Ocak 2026, Gece  
**ROS 2 Distro**: Humble  
**Gazebo**: Ignition Fortress/Harmonic  
**gz_ros2_control**: 0.7.17  

**Durum**: Controllers yükleniyor ama çakışma var. Robot henüz hareket etmiyor. Sonraki oturumda controller loading stratejisi düzeltilecek.
