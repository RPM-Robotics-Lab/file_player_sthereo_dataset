cd ..
git clone https://github.com/irapkaist/irp_sen_msgs.git
git clone https://github.com/swri-robotics/novatel_gps_driver.git

cd novatel_gps_driver
mv novatel_gps_msgs ../

rm -rf novatel_gps_driver
