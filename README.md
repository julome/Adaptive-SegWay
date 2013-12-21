Adaptive-Control-AegWay
========================

Adaptive predictive control with Arduino

El objetivo es la implementación de control adaptativo predictivo en un microcontrolador de bajo coste como Arduino.

Este es un ejemplo de control adaptativo predictivo implementado en un robot equilibrista utilizando un MPU6050 y un Atmega168 (Arduino Pro Mini).

En el desarrollo de este proyecto he utilizado un horizonte de predicción de 5 periodos, con un retraso de un periodo de control, un periodo de control de 14ms, las lecturas del giróscopo y el acelerómetro se realizan cada 1ms filtrando solo el acelerometro, para mas tarde cada 6ms pasar las medidas del acelerómetro por un filtro complementario tipo Kalman. Finalmente se obtiene una medida filtrada cada 12ms momento en que se llama a la función adaptativa predictiva. Se han desactivado todos los filtros en el MPU6050 lo que permite una gran velocidad de respuesta. 

Podemos comprobar como incluso el control adaptativo predictivo se adapta al ruido obteniendo un control satisfactorio del proceso. En este video se puede ver el resultado: http://youtu.be/K3qdtP572tM
