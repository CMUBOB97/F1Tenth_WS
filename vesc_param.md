[VESC parameter tuning info]

1. the file is located at src/f1tenth\_system/f1tenth\_stack/config/vesc.yaml

2. remember to reverse the sign of equation at line 100 in src/f1tenth\_system/vesc/vesc\_ackermann/src/vesc\_to\_odom.cpp

3. here are the gains:
- steering\_angle\_to\_servo\_offset: 0.4604
- steering\_angle\_to\_servo\_gain:

4. the teleop scale factors and axis assignments can be found in src/f1tenth\_system/f1tenth\_stack/config/joy\_teleop.yaml

