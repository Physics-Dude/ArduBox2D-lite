# ArduBox2D-lite
A modification of Box2D-lite with fixed-point numbers that can simulate multiple rigid bodies with collisions on the Arduboy.

At the moment this is a **proof of concept**. I havent seen a true 2D physics sim for the Arduboy (or any arduino micro) that supports everyhting that Box2D-lite does. I hope this can change that. 

***Shortest demo:***

https://user-images.githubusercontent.com/22563517/211410255-9f3151fe-15ec-4c7a-9000-74b680746565.mp4  


***What is this?***  
This is a program for the Arduboy and other small micros that simulates the physics of multiple rigid bodies in 2D space. Complete with complex collisions! It is a modification of Box2D-lite that uses Pharap's FixedPointsArduino library in place of all the floating point operations.  

***How?***  
I wish I knew... Im not a CS major. I just like physics. 
I did a CTRL+F for all instances of the word "float" and replaced them with "SQ7x8". I then spent a few hours fixing all the errors the compiler was throwing. Most of which were due to the lack of standard C++ libraries like \<map> and \<vector>. Adding mike-matera's ArduinoSTL library solved most of these. I was able to render about a dozen rigid bodies, but crashes happen if there are too many collisions.  

This was initially done in the Arduino IDE v1.8.19 with libraries available in the Library Manager.  

***Info:***  
Everything in the Box2D-lite library is still here except for "joints". I was fighting with space constraints of the ATMEGA32u4 prior to moving to fixed point math.  

No optimizations have been made. This is a simple adaption of Demo4 in the Box2D-lite sample program. Other demos not involving joints should still work as well. Visit the original Box2D-lite repasotory for more info.  

***Further reading:***  
- Required: Pharap's FixedPointsArduino: https://github.com/Pharap/FixedPointsArduino/  
- Required: mike-matera's ArduinoSTL: https://github.com/mike-matera/ArduinoSTL  
- Required: Arduboy2 (only using it for drawing) https://github.com/MLXXXp/Arduboy2  
- Not required, but uses physics code from: https://github.com/erincatto/box2d-lite  
- More info on Arduboy https://community.arduboy.com/t/documentation/7836
  
  
