ROS 2 ja sen asennus sujui ongelmitta seuraen ohjeita:
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Gazebo IGN ongelmia oli kun yritti asentaa VM joka ajoi Ubuntua, ongelmaksi koitui lopulta vmwaren gpu ajuri, kun hostilla oli Quadro RTX 3000, ongelma poistui kun gpu:lle sai native driverin ilman vm väliä. Toisaalta ignitionia ei tässä projektissa lopulta tarvinutkaan.

Gazebo 11 asennus sujui ongelmitta seuraen ohjeita:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
ja
https://ubuntu.com/blog/simulate-the-turtlebot3

Robotin rakennus oli helppoa ja siihen liittyvät asiat sai toimimaan helposti esim. samassa WLAN:issa olevien laitteiden välinen ssh yhteys ja muut vastaavat temput. esim. robotin ohjaus masterin näppäimistöllä ja robotti wireless modessa.
