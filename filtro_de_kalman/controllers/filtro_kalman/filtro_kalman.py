from controller import Robot, Keyboard, Lidar
from math import *
import numpy as np
import matplotlib.pyplot as plt
from numpy import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# lidar
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()
lidar_main_motor = robot.getDevice('LDS-01_main_motor')
lidar_secondary_motor = robot.getDevice('LDS-01_secondary_motor')
lidar_main_motor.setPosition(float('inf'))
lidar_secondary_motor.setPosition(float('inf'))
lidar_main_motor.setVelocity(30)
lidar_secondary_motor.setVelocity(60)

# keyboard
keyboard = Keyboard()
keyboard.enable(timestep)

# wheels
left = robot.getDevice('left wheel motor')
right = robot.getDevice('right wheel motor')
left.setPosition(float('inf'))
right.setPosition(float('inf'))
left.setVelocity(0)
right.setVelocity(0)
# encoder
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# variables and constants
raio = 0.033
distancia_rodas = 0.178
pose = [0,0,0] # x, y, theta
medidas = [0, 0] # esq, dir
ultimas_medidas = [0, 0] # esq, dir
distancias = [0, 0]
# mapa
estado_inicial = -2.64
mapa = [-1.6, -0.189, 1.16, 2.4 ] # posição central das três “portas” existentes
pose[0] = estado_inicial # atualiza como estado_inicial a posição x de pose

sigma_odometria = 0.2 # rad
sigma_lidar = 0.175 # meters
sigma_movimento = 0.002 # m

def gaussian(x, mean, sigma):
 return (1 / (sigma*sqrt(2*pi))) * exp(-((x-mean)**2) / (2*sigma**2))

# update function
def update():
 medidas[0] = left_encoder.getValue()
 medidas[1] = right_encoder.getValue()
 diff = medidas[0] - ultimas_medidas[0] # conta quanto a roda LEFT girou desde a última medida (rad)
 distancias[0] = diff * raio + random.normal(0,0.002) # determina distância percorrida em metros e adiciona um pequeno erro
 ultimas_medidas[0] = medidas[0]
 diff = medidas[1] - ultimas_medidas[1] # conta quanto a roda LEFT girou desde a última medida (rad)
 distancias[1] = diff * raio + random.normal(0,0.002) # determina distância percorrida em metros + pequeno erro
 ultimas_medidas[1] = medidas[1]
 # ## cálculo da dist linear e angular percorrida no timestep
 deltaS = (distancias[0] + distancias[1]) / 2.0
 deltaTheta = (distancias[1] - distancias[0]) / distancia_rodas
 pose[2] = (pose[2] + deltaTheta) % 6.28 # atualiza o valor Theta (diferença da divisão por 2π)

 # decomposição x e y baseado no ângulo
 deltaSx = deltaS * cos(pose[2])
 deltaSy = deltaS * sin(pose[2])

 # atualização acumulativa da posição x e y
 pose[0] = pose[0] + deltaSx # atualiza x
 pose[1] = pose[1] + deltaSy # atualiza y

 print("Postura:", pose)
 
 # initial graph # em roxo tudo que for relativo ao gráfico de plotagem
x = np.linspace(-4.5, 4.5, 500) # cria um vetor x de 500 valores entre -4.5 e 4.5
y = np.zeros(500) # cria um vetor y de 500 valores zeros
y2 = np.zeros(500)
y3 = np.zeros(500)
fig, ax = plt.subplots()
# main loop ------------------------------------------------------------------------------------------
controle = 0
cont = 0
porta = 0 # robô começa em frente antes da porta 0

while robot.step(timestep) != -1:
#PLOTAR GAUSSIANA DO ROBÔ
 if cont % 4 == 0: # a cada 4 passos, plotar em preto “b” a gaussiana da posição do robô em x (pose[0])
     for i in range(len(x)):
         y[i] = gaussian(x[i], pose[0], sigma_movimento)
     ax.clear()
     ax.set_ylim([0, 4])
     ax.plot(x, y, color="b")
     plt.pause(0.1)
 leitura = lidar.getRangeImage()
 print("Leitura 72: ",leitura[72])
 print("Leitura 108: ",leitura[108])
 # print("Leitura 109: ",leitura[1])
 # print("Leitura:", leitura)
 
 
 update() # atualiza a nova pose do robô
 key = keyboard.getKey() # movimentação do robô pelo teclado
 if key == ord("W"): # W para frente com velocidade 4
     left.setVelocity(4)
     right.setVelocity(4)
     controle = 1 # controle = 1 indica que o robô está indo para frente

 elif key == ord("A"): # A gira para esquerda com velocidade 2
     left.setVelocity(-2)
     right.setVelocity(2)

 elif key == ord("D"): # D gira para a direita com velocidade 2
     left.setVelocity(2)
     right.setVelocity(-2)

 elif key == ord("S"): # S = stop the robot
     left.setVelocity(0)
     right.setVelocity(0)

 if controle == 1:
     sigma_movimento = sigma_movimento + 0.002 # se movimento reto, aumenta a incerteza da posição em 0.002  
 # if leitura[72] == inf and leitura[108] == inf: # se a leitura indicar em frente a uma porta
     # left.setVelocity(0)
     # right.setVelocity(0)
 
 if leitura[72] <1 and leitura[108] <1: # se a leitura indicar em frente a uma porta
     left.setVelocity(0)
     right.setVelocity(0)

     media_nova = (mapa[porta]*sigma_movimento + pose[0]*sigma_lidar) / (sigma_movimento+sigma_lidar)
     sigma_novo = 1 / (1/sigma_movimento + 1/sigma_lidar)
     pose[0] = media_nova # a nova posição x do robô
     sigma_movimento = sigma_novo # novo erro gaussiano do robô
     for i in range(len(x)): y2[i] = gaussian(x[i], mapa[porta], sigma_lidar)
     ax.plot(x, y2, color="r")
     plt.pause(0.1) # plota em vermelho “r” a gaussiana da leitura do laser com relação à porta
     robot.step(3000)
     for i in range(len(x)): y3[i] = gaussian(x[i], media_nova, sigma_novo)
     ax.plot(x, y3, color="g")
     plt.pause(0.1) # plota em verde “g” a gaussiana nova após interpolação das duas gaussianas.
     robot.step(3000)
     left.setVelocity(4)
     right.setVelocity(4)
     if porta == 0: porta = 1 # altera para a próxima porta 0 → 1 ; 1 → 2
     elif porta == 1: porta = 2
    
     robot.step(1000)

 cont += 1
