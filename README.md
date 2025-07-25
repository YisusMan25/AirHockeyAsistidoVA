# AirHockeyAsistidoVA
El proyecto consiste en el desarrollo de un sistema autónomo de Air Hockey que combina visión artificial, control robótico y procesamiento en tiempo real para simular un jugador automatizado. La plataforma está compuesta por una mesa de Air Hockey modificada, un robot de dos ejes (XY) con estructura impresa en 3D, y una unidad de control basada en Arduino Uno con shield RAMPS V3 utilizando el firmware GRBL.

El sistema utiliza una cámara para detectar la posición de un disco rojo flotando sobre la mesa, gracias al aire generado por ventiladores de 12V colocados en la parte inferior. A través de algoritmos desarrollados en Python con OpenCV, el sistema identifica el disco, predice su trayectoria y traduce sus coordenadas a posiciones físicas que el robot puede alcanzar. La comunicación con el actuador se realiza mediante comandos G-code enviados directamente al firmware GRBL.

Se realizaron adecuaciones específicas como el puenteo del driver Z para duplicar el movimiento del eje Y en ambos motores laterales, y la inversión del giro de uno de ellos para lograr sincronía. La calibración fina del sistema se efectuó usando el software Universal G-code Sender (UGS).

El resultado es un sistema capaz de responder en tiempo real al movimiento del disco, desplazando un actuador sobre la superficie de juego con precisión y velocidad, imitando el comportamiento de un jugador humano. Esta plataforma constituye una base sólida para el desarrollo de futuras mejoras como estrategias ofensivas, aprendizaje automático y operación completamente autónoma.
