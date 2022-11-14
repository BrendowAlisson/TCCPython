#bibliotecas
import cv2
import numpy as np
import mediapipe as mp
import math
import random
from paho.mqtt import client as mqtt_client
import json

def modulo(vetor):
    mod = math.sqrt(vetor[0]**2 + vetor[1]**2 + vetor[2]**2)
    return mod

def dedo(lista, inicio, fim):
    x = (lista[fim][1] - lista[inicio][1])
    y = (lista[fim][2] - lista[inicio][2])
    z = (lista[fim][3] - lista[inicio][3])
    vetor = np.array([x, y, z])
    return vetor

def produto_vetorial(lista, inicio, fim1, fim2):
    x1 = (lista[fim1][1] - lista[inicio][1])
    y1 = (lista[fim1][2] - lista[inicio][2])
    z1 = (lista[fim1][3] - lista[inicio][3])

    x2 = (lista[fim2][1] - lista[inicio][1])
    y2 = (lista[fim2][2] - lista[inicio][2])
    z2 = (lista[fim2][3] - lista[inicio][3])

    vetor1 = np.array([x1, y1, z1])
    vetor2 = np.array([x2, y2, z2])

    resultado = np.cross(vetor2, vetor1, axisa=- 1, axisb=- 1, axisc=- 1, axis=None)

    return resultado

def angulo(vetor0, vetor1):
    normres = vetor1 / np.linalg.norm(vetor1)
    norm0 = vetor0 / np.linalg.norm(vetor0)
    mod0 = modulo(norm0)
    mod1 = modulo(normres)
    resultado = np.dot(normres, norm0) / (mod0 * mod1)
    angle_degrees = math.acos(resultado) * 180 / math.pi
    return angle_degrees

def servo_angle(teta, dedo):
    global tres
    if dedo == 'polegar':
        servoAngle = int((180 - (100 - teta) * 2))
        if servoAngle < 0:
            servoAngle = 0
        elif servoAngle > 180:
            servoAngle = 180
    else:
        if dedo == 'meio':
            graus = 200
        if dedo == 'mindinho' and tres == True:
            graus = -1000
        else:
            graus = 180
        servoAngle = int((graus - (100 - teta) * 2))
        if servoAngle < 10:
            servoAngle = 10
        elif servoAngle > 180:
            servoAngle = 180
    return servoAngle

def media(cliente):
    global mediaAnteriorIndicador, movIndicador
    global mediaAnteriorMeio, movMeio
    global mediaAnteriorAnel, movAnel
    global mediaAnteriorMindinho, movMindinho
    global mediaAnteriorPolegar, movPolegar
    global dedo0, dedo1, dedo2, dedo3, dedo4
    limite = 1
    intervalo = 10

    if len(movIndicador) == limite:
        mediaInd = int(np.mean(movIndicador))
        dedo3 = True
        if mediaAnteriorIndicador + intervalo < mediaInd or mediaAnteriorIndicador - intervalo > mediaInd:
            mediaAnteriorIndicador = mediaInd
            movIndicador = []
    elif len(movIndicador) > limite:
            movIndicador = []

    if len(movMeio) == limite:
        mediaMeio = int(np.mean(movMeio))
        dedo2 = True
        if mediaAnteriorMeio + intervalo < mediaMeio or mediaAnteriorMeio - intervalo > mediaMeio:
            mediaAnteriorMeio = mediaMeio
            movMeio = []
    elif len(movMeio) > limite:
            movMeio = []

    if len(movAnel) == limite:
        mediaAnel = int(np.mean(movAnel))
        dedo1 = True
        if mediaAnteriorAnel + intervalo < mediaAnel or mediaAnteriorAnel - intervalo > mediaAnel :
            mediaAnteriorAnel = mediaAnel
            movAnel = []
    elif len(movAnel) > limite:
            movAnel = []

    if len(movMindinho) == limite:
        mediaMindinho = int(np.mean(movMindinho))
        dedo0 = True
        if mediaAnteriorMindinho + intervalo < mediaMindinho or mediaAnteriorMindinho - intervalo > mediaMindinho:
            mediaAnteriorMindinho = mediaMindinho
            movMindinho = []
    elif len(movMindinho) > limite:
        movMindinho = []

    if len(movPolegar) == limite:
        mediaPolegar = int(np.mean(movPolegar))
        dedo4 = True
        if mediaAnteriorPolegar + intervalo < mediaPolegar or mediaAnteriorPolegar - intervalo > mediaPolegar:
            mediaAnteriorPolegar = mediaPolegar
            movPolegar = []
    elif len(movPolegar) > limite:
        movPolegar = []

    if dedo0 == True and dedo1 == True and dedo2 == True and dedo3 == True and dedo4 == True:
        msg = {"servoAngleLittle":mediaAnteriorMindinho,"servoAngleRing":mediaAnteriorAnel,"servoAngleMiddle":mediaAnteriorMeio,"servoAngleNail":mediaAnteriorIndicador,"servoAngleThumb":mediaAnteriorPolegar}
        json_object = json.dumps(msg, indent = 4)

        #msg = f'mindinho:{mediaAnteriorMindinho},anelar:{mediaAnteriorAnel},meio:{mediaAnteriorMeio},indicador:{mediaAnteriorIndicador},polegar:{mediaAnteriorPolegar}'
        envio = cliente.publish(topic, json_object, 0)
        if envio[0] == 0:
            print(msg)
        else:
            print('Não enviou')
        dedo0 = False
        dedo1 = False
        dedo2 = False
        dedo3 = False
        dedo4 = False

def verificar_tres(x,y):
    verificacao = math.hypot(x, y)
    if verificacao < 30:
        return True
    else:
        return False

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print('Conexão bem-sucedida')
        else:
            print(f'erro{rc}')
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client



#utilizar a biblioteca para reconhecer a mão
mpMao = mp.solutions.hands
maos = mpMao.Hands(max_num_hands=1, min_detection_confidence=0.7) #reconhecimento de uma mão e min_detection_confidence são quantos % para a detecção ser considerada sucesso
mpDesenho = mp.solutions.drawing_utils #desenhar os vértices

wCam, hCam = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)

movIndicador = []
mediaAnteriorIndicador = 0

movMeio = []
mediaAnteriorMeio = 0

movAnel = []
mediaAnteriorAnel = 0

movMindinho = []
mediaAnteriorMindinho = 0

movPolegar = []
mediaAnteriorPolegar = 0

tres = False
dedo0 = False
dedo1 = False
dedo2 = False
dedo3 = False
dedo4 = False

broker = 'broker.hivemq.com'
port = 1883
topic = "/topic/esp32/mao"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
client = connect_mqtt()
while True:
    sucess, img = cap.read()
    x, y, c = img.shape
    frameBGR = cv2.flip(img, 1)
    frameRGB = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)
    resultado = maos.process(frameRGB)
    lista = []
    lista2 = []
    # Checa se alguma mão foi detectada ou não
    if resultado.multi_hand_landmarks:
        # aqui me da um dicionario de landmark {}, landmark são as marcacoes
        for maosCoordenadas in resultado.multi_hand_landmarks:
            # aqui vai me dar as coordenadas dentro do dicionário landmark
            for id, coordenadas in enumerate(maosCoordenadas.landmark):
                # multiplicamos por x e y dea imagem pq as coordenadas vem normalizadas
                coordenadax = int(coordenadas.x * y)
                coordenaday = int(coordenadas.y * x)
                lista.append([id, coordenadas.x, coordenadas.y, coordenadas.z])
                lista2.append([id, coordenadax, coordenaday, coordenadas.z])
                #isso faz desenhar os circulos na coordenadas da mão
            #mpDesenho.draw_landmarks(frameBGR, maosCoordenadas, mpMao.HAND_CONNECTIONS)
            for i in range(0, 21):
                cv2.circle(frameBGR, (lista2[i][1], lista2[i][2]), 5, (0, 0, 0), cv2.FILLED)
            #cv2.line(frameBGR,(lista2[8][1],lista2[8][2]),(lista2[5][1],lista2[5][2]),(255,0,0),3)
            #cv2.line(frameBGR, (lista2[5][1], lista2[5][2]), (lista2[0][1], lista2[0][2]), (255, 0, 255), 3)
            #cv2.line(frameBGR, (lista2[17][1], lista2[17][2]), (lista2[0][1], lista2[0][2]), (255, 255, 0), 3)

            indicador = dedo(lista, 5, 8)
            meio = dedo(lista, 9, 12)
            anelar = dedo(lista, 13, 16)
            mindinho = dedo(lista, 17, 20)
            polegar = dedo(lista, 4, 17)

            vetorNormal = produto_vetorial(lista, 0, 5, 17)

            tetaIndicador = angulo(indicador, vetorNormal)
            tetaMeio = angulo(meio, vetorNormal)
            tetaAnelar = angulo(anelar, vetorNormal)
            tetaMindinho = angulo(mindinho, vetorNormal)
            tetaPolegar = angulo(polegar, vetorNormal)

            x = lista2[20][1] - lista2[4][1]
            y = lista2[20][2] - lista2[4][2]

            tres = verificar_tres(x, y)

            servoIndicador = servo_angle(tetaIndicador, 'indicador')
            servoMeio = servo_angle(tetaMeio, 'meio')
            servoAnelar = servo_angle(tetaAnelar, 'anelar')
            servoMindinho = servo_angle(tetaMindinho, 'mindinho')
            servoPolegar = servo_angle(tetaPolegar, 'polegar')

            movIndicador.append(servoIndicador)
            movMeio.append(servoMeio)
            movAnel.append(servoAnelar)
            movMindinho.append(servoMindinho)
            movPolegar.append(servoPolegar)

            client.loop_start()
            media(client)



    cv2.imshow('saida', frameBGR)

    if cv2.waitKey(1) == ord('q'):
        break


cv2.destroyAllWindows()