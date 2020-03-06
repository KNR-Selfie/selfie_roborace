
# speed_controller
`speed_controller` to paczka, wyznaczająca optymalną prędkość w oparciu o kąt odchylenia i odległość od optymalnego toru.
Przymuje trzy argumenty.
- określający stromiznę krzywej (a)
- określający wpływ kąta na prędkość (b)
- określający wpływ odległości na prędkość (c)
# Wersja alfa
![equation](http://www.sciweavers.org/upload/Tex2Img_1554547543/render.png)
## `speed_controller`

## Subscribed topics

`position_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

`heading_offset`([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))

## Published topics
`target_speed` ([std_msgs/Float64 Message](http://docs.ros.org/lunar/api/std_msgs/html/msg/Float64.html))

## Parameters

- `max_speed` - maksymalna prędkość samochodu wyrażona w m/s (double)
- `speed` - parametr kalibrujący wzrost prędkości w dziedzinie (double)
- `angle` - parametr kalibrujący "ważność" wartości kąta (double)
- `distance` - parametr kalibrujący "ważność" dystansu (double)
