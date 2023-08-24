
from controller import Robot, Keyboard
from math import cos, sin



robot = Robot()
teclado = Keyboard()
timestep = int(robot.getBasicTimeStep())
left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")

left.setPosition(float('inf'))
right.setPosition(float('inf'))

# setup encoder
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

#teclado
teclado.enable(timestep)

raio =0.033 # metros
distancia_rodas = 0.178 # metros
medidas = [0, 0]
ultimas_medidas = [0,0]
distancia = [0,0]
postura = [0,0,0] #x ,y , theta

def odometria():
    medidas[0] = left_encoder.getValue();
    medidas[1] = right_encoder.getValue();
    for i in range(2):
        diff = medidas[i] - ultimas_medidas[i]
        distancia[i] = diff * raio
    #calculo da velocidade linear e angular
    deltaS = (distancia[0]+distancia[1])/2
    deltaTheta = (distancia[1] - distancia[0])/distancia_rodas
    
    postura[2] = (postura[2] + deltaTheta) % 6.28
    
    deltaSx = deltaS * cos(postura[2])
    deltaSy = deltaS * sin(postura[2])
    
    postura[0] = postura[0] + deltaSx
    postura[1] = postura[1] + deltaSy
    
    for i in range(2):
        ultimas_medidas[i] = medidas[i]
        
    print(postura)

left.setVelocity(0)
right.setVelocity(0)
    

while robot.step(timestep) != -1:
    
    odometria()
    tecla = teclado.getKey()
    if tecla == ord('W'):
        left.setVelocity(1)
        right.setVelocity(1)
    elif tecla == ord('A'):
        left.setVelocity(-1)
        right.setVelocity(1)
    elif tecla == ord('D'):
        left.setVelocity(1)
        right.setVelocity(-1)
    elif tecla == ord('S'):
        left.setVelocity(0)
        right.setVelocity(0)
        
       
    
    
    pass


