from cmath import atan, sqrt
import cmath
import numpy
from PIL import Image
from multiprocessing import Process, Array

# classe para passar os dados quando houver algum hit
class rayhit:
    def __init__(self, hitObj,hitPoint, hitNormal, hitDistance, color, ray):
        self.hitObj = hitObj
        self.hitPoint = hitPoint
        self.hitNormal = hitNormal
        self.hitDistance = hitDistance
        self.color = color
        self.ray = ray

# classe principal dos objetos da cena
class scene_object:
    def __init__(self, position = [0,0,0], color = (255,0,0), ka=1, kd=1, ks=1, phongN=1, kr=0, kt=0, refN = 1):
        self.position = position
        self.color = color        
        self.ka = ka
        self.kd = kd
        self.ks = ks
        self.phongN = phongN
        self.kr = kr
        self.kt = kt
        self.refN = refN
    
    def getColor(self,p):
        return self.color

    # retorna a normal no ponto p
    def getNormal(self, p):
        return
    
    # retorna 0 se não houver hit, se houver retorna um rayhit
    def intersection(self, origin, direction):
        return

# classe de objeto especial piso quadriculado usada apenas para testes proprios (similar ao plano)
class piso(scene_object):
    def __init__(self, position = [0,0,0], normal = [0,1,0], color = (255,0,0), ka=1, kd=1, ks=1, phongN=1, kr=0, kt=0, refN = 1):
        self.normal = normalized(normal)
        super().__init__(position, color, ka, kd, ks, phongN, kr, kt, refN)
    
    def getNormal(self, p):
        return self.normal
    
    # getcolor especial para formar o quadriculado
    def getColor(self, p):
        f = .025
        if numpy.floor(p[0].real * f) % 2 ==  numpy.floor(p[1].real * f) % 2:
            return (0,0,0)
        else:
            return (255,255,255)

    def intersection(self, origin, direction):
        #formula para interseção demonstrada no scratchapixel.com        
        ldotn = numpy.dot(normalized(direction),normalized(self.normal))
        if ldotn >= 0.000:
            return 0
        
        t = numpy.dot((self.position - origin), self.normal) / ldotn

        if t<0:
            return 0
        else:
            hitPoint = origin + direction * t
            normal = self.getNormal(hitPoint)
            color = self.getColor(hitPoint)
            return rayhit(self, hitPoint, normal, t, color, hitPoint - origin)
    

# classe do objeto: plano
class plane(scene_object):
    def __init__(self, position = [0,0,0], normal = [0,1,0], color = (255,0,0), ka=1, kd=1, ks=1, phongN=1, kr=0, kt=0, refN = 1):
        self.normal = normalized(normal)
        super().__init__(position, color, ka, kd, ks, phongN, kr, kt, refN)
    
    def getNormal(self, p):
        return self.normal
    
    def getColor(self, p):
        return super().getColor(p)

    def intersection(self, origin, direction):
        #formula para interseção demonstrada no scratchapixel.com        
        ldotn = numpy.dot(normalized(direction),normalized(self.normal))

        # nao ha colisao se o plano nao esta de frente para o raio
        if ldotn >= 0.000:
            return 0
        
        t = numpy.dot((self.position - origin), self.normal) / ldotn
        
        # nao ha colisao
        if t<0:
            return 0
        else: # colisao
            hitPoint = origin + direction * t
            normal = self.getNormal(hitPoint)
            color = self.getColor(hitPoint)
            return rayhit(self, hitPoint, normal, t, color, hitPoint - origin)

# classe do objeto: esfera
class sphere(scene_object):

    def __init__(self, position = [0,0,0], radius = 1, color = (255,0,0), ka=1, kd=1, ks=1, phongN=1, kr=0, kt=0, refN = 1):
        self.radius = radius
        super().__init__(position, color, ka, kd, ks, phongN, kr, kt, refN)
    
    def getNormal(self, p):
        return normalized(p - self.position)
    
    def getColor(self, p):
        return super().getColor(p)
    
    def intersection(self, origin, direction):
        #formula para interseção demonstrada no stratchapixel.com
        l = self.position - origin
        
        tca = numpy.dot(l,direction)

        if tca < 0:
            return 0
        
        pitagoras = pow(numpy.linalg.norm(l) ,2) - pow(tca,2)
        if pitagoras < 0:
            return 0
        d = sqrt(pitagoras)
        
        if d.real < 0.0:
            return 0
        elif d.real > self.radius:
            return 0
        else:
            thc = sqrt(pow(self.radius,2) - pow(d,2))

            # buscamos a colisao mais proxima
            hitDist = numpy.minimum(tca - thc, tca + thc)
            
            # mas, se estamos dentro da esfera significa que o raio acabou de entrar na esfera, logo usamos a colisao mais distance
            if numpy.linalg.norm(l) < self.radius: 
                hitDist = numpy.maximum(tca-thc,tca+thc)

            hitPoint = origin + direction * hitDist

            normal = self.getNormal(hitPoint)

            color = self.getColor(hitPoint)
            
            return rayhit(self, hitPoint, normal, hitDist, color, hitPoint - origin)

# classe da cena, vai guardar os objetos
class scene_main:
    def __init__(self):
        
        # criação de objetos para popular a cena
        self.objs = []
        self.lights = []
        self.bg_color = (0,0,0)
        self.ambientLight = (0,0,0)
    
    def setBackground_Color(self, color):
        self.bg_color = color
    
    def getBackground_Color(self):
        return self.bg_color

    def addSphere(self, position, radius, color, ka, kd, ks, phongN, kr, kt, refN):
        self.objs.append(sphere(position, radius, color, ka, kd, ks, phongN, kr, kt, refN))
    
    def addPlane(self, position, normal, color, ka, kd, ks, phongN, kr, kt, refN):
        self.objs.append(plane(position, normal, color, ka, kd, ks, phongN, kr, kt, refN))

    def addPiso(self, position, normal, color, ka, kd, ks, phongN, kr, kt, refN):
        self.objs.append(piso(position, normal, color, ka, kd, ks, phongN, kr, kt, refN))

    def addPointLight(self, position, color):
        self.lights.append(pointLight(position, color))
    
    def setAmbientLight(self, color):
        self.ambientLight = color
    

# classe das luzes que vao popular scene
class light:
    def __init__(self, position, color):
        self.position = position
        self.color = color

# classe da luz especifica do tipo pointLight
class pointLight(light):

    def __init__(self, position, color):
        super().__init__(position, color)

# função principal para criar a imagem test.png com o resultado
def render(res_h, res_v, pxl_size,d,cam_pos,cam_forward,cam_up, scene, max_depth):
    # cria um Image todo preto
    img = Image.new('RGB', (res_h,res_v), color = (0,0,0))
    
    # acha o vetor da camera que aponta para a sua direita
    cam_right = numpy.asarray(numpy.cross(cam_up, cam_forward))
    
    # centro do pixel mais a esquerda e acima
    topleft = cam_pos + cam_forward * d + (cam_up *  (res_v - 1) - cam_right * (res_h - 1)) * pxl_size * 0.5
    
    # quantas threads serão usadas (process)
    thread_count = 12
    
    # o range de cada thread horizontalmente
    xranges = []

    for i in range(thread_count):
        xranges.append(int(i * (res_h / thread_count)))
    xranges.append(res_h)
    
    # lista das threads
    all_threads = []
    # lista das cores de cada thread
    ars = []

    # criacao das threads definindo o espaco horizontal de cada uma
    for t in range(thread_count):
        ars.append(Array("i", range(res_v * (xranges[t + 1] - xranges[t]))))
        ars.append(Array("i", range(res_v * (xranges[t + 1] - xranges[t]))))
        ars.append(Array("i", range(res_v * (xranges[t + 1] - xranges[t]))))

        all_threads.append(Process(target=thread_render, args=(cam_pos,cam_up,cam_right,topleft,pxl_size,scene,xranges[t],xranges[t+1],0,res_v, 
            max_depth, ars[t*3],ars[t*3+1], ars[t*3+2]),daemon=True))
    
    #iniciar as threads
    for x in all_threads:
        x.start()
    
    # esperar todas as threads concluirem
    for x in all_threads:
        x.join()

    # gravacao dos valores calculados pelas threads na imagem 
    for i in range(thread_count):
        for x in range(xranges[i + 1] - xranges[i]):
            for y in range(res_v):
                c = (ars[i * 3 + 0][x * res_v + y] ,ars[i * 3 + 1][x * res_v + y] ,ars[i * 3 + 2][x * res_v + y])              
                img.putpixel((xranges[i] + x, y),c)       

    img.save('test.png')
    print("imagem salva")


# funcao que vai obter os dados de pixels em um segmento da imagem renderizada
# os intervalos x0, x1 sao usados usados
def thread_render(cam_pos, cam_up, cam_right, topleft, pxl_size, scene, x0, x1, y0, y1, max_depth, arsR,arsG,arsB):
    
    for x_ in range(x1-x0):
        x = x_ + x0
        
        for y_ in range(y1-y0):
            y = y_ + y0

            ray_dir = normalized((topleft + (cam_up * -y + cam_right * x) * pxl_size) - cam_pos)
            # raycast e feito a partir da camera com a direcao do raio, max_depth e a profundida da recursao(raios secundarios de reflexao e refracao)
            c = colorDenormalize(cast(cam_pos,ray_dir, scene, max_depth))
            arsR[x_ * y1 + y_] = c[0]
            arsG[x_ * y1 + y_] = c[1]
            arsB[x_ * y1 + y_] = c[2]
    
    print("thread end")


# funcao cast sera chamada recursivamente, a entrada counter definira quantas recursoes serao feitas
def cast(origin, direction,scene, counter):
    color = colorNormalize(scene.getBackground_Color())
    hit = trace(origin,direction,scene)
    if hit:
        color = shade(hit, scene, counter)
        
    return (color)


def trace(origin, direction, scene:scene_main):
    hit = 0

    # buscamos intersecoes com todos os objetos e retornamos a intersecao mais proxima ao ponto de origem do raio
    for i in range(len(scene.objs)):

        hitCheck = scene.objs[i].intersection(origin,direction)
         
        if hitCheck != 0 and (hit == 0 or hitCheck.hitDistance < hit.hitDistance):
            hit = hitCheck
    
    return hit

# funcao shade sera chamada recursivamente, a entrada counter definira quantas recursoes serao feitas
def shade(hit:rayhit, scene:scene_main, counter):
    # cor do objeto
    color_difuse = colorNormalize(hit.color)
    # cor do pixel iniciada com a luz ambiente
    color =  colorScale(colorMul(color_difuse, colorNormalize(scene.ambientLight)), hit.hitObj.ka)

    # para cada luz na cena calcular a cor
    for light in scene.lights:
        color_light = colorNormalize(light.color)
        l = light.position - hit.hitPoint
        lDist = numpy.linalg.norm(l)
        l = normalized(l)

        ndotl = numpy.dot(hit.hitNormal, l).real
        
        
        # se recebe luz
        if ndotl > 0:
            shadowHit = trace(hit.hitPoint + l *0.00001, l, scene)
            if shadowHit !=0 and shadowHit.hitDistance < lDist:
                continue
            
            # cor difusa
            color = colorSum(color, colorScale(colorMul(color_light, color_difuse), ndotl * hit.hitObj.kd))
            
            rj = 2 * ndotl * hit.hitNormal - l

            view = normalized(-hit.ray)
            rjdotview = numpy.dot(rj,view).real
            if rjdotview < 0:
                rjdotview = 0
            
            # cor especular
            color = colorSum(color, colorScale(color_light , hit.hitObj.ks * numpy.power(rjdotview, hit.hitObj.phongN)))

    # contador de rays recursivos
    if counter > 0:
        # refracao
        kr = hit.hitObj.kr
        if hit.hitObj.kt > 0:
            view = normalized(hit.ray)
            rayDir = refract(view, normalized(hit.hitNormal), hit.hitObj.refN)
            
            if numpy.isscalar(rayDir) == False: # se ha refracao
                # cast recursivo da refracao
                refColor = cast(hit.hitPoint + rayDir * 0.00001, rayDir, scene, counter-1)
                # soma da cor da refracao
                color = colorSum(color,colorScale(refColor, hit.hitObj.kt))
            else: # se nao ha refracao
                kr = 1
        
        #reflexao
        if kr > 0:
            view = normalized(hit.ray)
            rayDir = reflect(view, hit.hitNormal)
            # cast recursivo da reflexao
            refColor = cast(hit.hitPoint + rayDir * 0.00001, rayDir, scene, counter-1)
            # soma da cor da reflexao
            color = colorSum(color,colorScale(refColor, kr))
    

    return color

# funcao que retorna um vetor normalizado
def normalized(vec):
    n = numpy.linalg.norm(vec)
    if n ==0:
        return vec
    else:
        return vec / n

# função para refletir um raio
def reflect(vec, normal):
    n = normalized(normal)
    return numpy.dot(vec, n) * n * -2 + vec

# refracao: vector = vetor incidencia // normal = normal da superficie // n = n1 / n2 (coeficientes) 
def refract(vec, normal, n):

    w = -vec

    if numpy.dot(w,normal)>0:   # caso entrando no objeto
        ndotw = numpy.dot(normal,w)

        delta = 1 - (1/(n*n)) *(1-ndotw*ndotw)

        if delta < 0:
            return -1
        else:
            t = - (1/n) * w - (numpy.sqrt(delta) - (1/n) * ndotw) * normal
            return t
    else:                       # caso saindo do objeto        
        normal1 = -normal
        ndotw = numpy.dot(normal1,w)
        
        n1 = 1/n

        delta = 1 - (1/(n1*n1)) *(1-ndotw*ndotw)

        if(delta<0):
            return -1
        else:
            t =  - (1/n1) * w - (numpy.sqrt(delta)-(1/n1) * ndotw) * normal1
            return t

# multiplica cores (0-1.0)
def colorMul(color1, color2):
    r1 = color1[0]
    g1 = color1[1]
    b1 = color1[2]
    
    r2 = color2[0]
    g2 = color2[1]
    b2 = color2[2]

    return (r1 * r2, g1 * g2, b1 * b2)

# multiplica cor por um escalar (0-1.0)
def colorScale(color, f):
    return (color[0] * f, color[1] * f, color[2] * f)

# soma duas cores (0-1.0)
def colorSum(color1, color2):
    r1 = color1[0]
    g1 = color1[1]
    b1 = color1[2]
    
    r2 = color2[0]
    g2 = color2[1]
    b2 = color2[2]

    return (r1+r2, g1+g2, b1+b2)

# passa a cor de (0 - 255) para (0 - 1.0)
def colorNormalize(color):
    return (float(color[0]) / 255.0, float(color[1]) / 255.0, float(color[2]) / 255.0)

# passa a cor de (0 - 1.0) para (0 - 255)
def colorDenormalize(color):
    f = max(1,*color)
    return (int(color[0] * 255.0/f), int(color[1] * 255.0/f), int(color[2] * 255.0/f))



if __name__ == '__main__':
    # nova cena e criada que guardara os objetos e luzes
    new_scene = scene_main()

    # multiplicador das coordenadas, para ajustar as entradas ao espaco
    xyz_coord = (1,-1,1)

    # LEITURA DOS INPUTS
    with open("input.txt") as f:
        inputs = f.read().split()

    index = 0

    res_vertical = int(inputs[index])
    index +=1
    res_horizontal = int(inputs[index])
    index +=1
    size_pixel = float(inputs[index])
    index +=1
    cam_dist = float(inputs[index])
    index +=1
    cam_pos_x = float(inputs[index])
    index +=1
    cam_pos_y = float(inputs[index])
    index +=1
    cam_pos_z = float(inputs[index])
    index +=1
    cam_forward_x = float(inputs[index])
    index +=1
    cam_forward_y = float(inputs[index])
    index +=1
    cam_forward_z = float(inputs[index])
    index +=1
    cam_up_x = float(inputs[index])
    index +=1
    cam_up_y = float(inputs[index])
    index +=1
    cam_up_z = float(inputs[index])
    index +=1
    bg_color_r = int(inputs[index])
    index +=1
    bg_color_g = int(inputs[index])
    index +=1
    bg_color_b = int(inputs[index])
    index +=1
    max_depth = int(inputs[index])
    index +=1
    k_obj = int(inputs[index])
    index +=1

    new_scene.setBackground_Color((bg_color_r,bg_color_g,bg_color_b))

    cam_pos = numpy.array([cam_pos_x  * xyz_coord[0], cam_pos_y  * xyz_coord[1], cam_pos_z * xyz_coord[2]])
    cam_forward = numpy.array([cam_forward_x * xyz_coord[0], cam_forward_y * xyz_coord[1], cam_forward_z * xyz_coord[2]]) - cam_pos
    cam_up = numpy.array([cam_up_x * xyz_coord[0], cam_up_y * xyz_coord[1], cam_up_z * xyz_coord[2]])

    cam_forward = normalized(cam_forward)
    cam_up = normalized(cam_up - numpy.dot(cam_forward, cam_up) * cam_forward)

    for i in range(k_obj):
        color_r = int(inputs[index])
        index +=1
        color_g = int(inputs[index])
        index +=1
        color_b = int(inputs[index])
        index +=1
        color = (color_r, color_g, color_b)

        ka = float(inputs[index])
        index +=1
        kd = float(inputs[index])
        index +=1
        ks = float(inputs[index])
        index +=1
        phongN = float(inputs[index])
        index +=1

        kr = float(inputs[index])
        index +=1
        kt = float(inputs[index])
        index +=1
        refN = float(inputs[index])
        index +=1

        obj_select = inputs[index]
        index +=1

        pos_x = float(inputs[index])
        index +=1
        pos_y = float(inputs[index])
        index +=1
        pos_z = float(inputs[index])
        index +=1

        position = numpy.array([pos_x * xyz_coord[0], pos_y * xyz_coord[1], pos_z * xyz_coord[2]])

        if obj_select == '*':
            radius = float(inputs[index])
            index +=1

            new_scene.addSphere(position, radius, color, ka, kd, ks, phongN, kr, kt, refN)
        elif obj_select == '/':
            normal_x = float(inputs[index])
            index +=1
            normal_y = float(inputs[index])
            index +=1
            normal_z = float(inputs[index])
            index +=1

            normal = normalized([normal_x * xyz_coord[0], normal_y * xyz_coord[1], normal_z * xyz_coord[2]])

            new_scene.addPlane(position, normal, color, ka, kd, ks, phongN, kr, kt, refN)
        else:
            normal_x = float(inputs[index])
            index +=1
            normal_y = float(inputs[index])
            index +=1
            normal_z = float(inputs[index])
            index +=1

            normal = normalized([normal_x * xyz_coord[0], normal_y * xyz_coord[1], normal_z * xyz_coord[2]])

            new_scene.addPiso(position, normal, color, ka, kd, ks, phongN, kr, kt, refN)


    cAmb_r = int(inputs[index])
    index +=1
    cAmb_g = int(inputs[index])
    index +=1
    cAmb_b = int(inputs[index])
    index +=1

    new_scene.setAmbientLight((cAmb_r,cAmb_g,cAmb_b))

    k_pl = int(inputs[index])
    index +=1

    for i in range(k_pl):
        color_r = int(inputs[index])
        index +=1
        color_g = int(inputs[index])
        index +=1
        color_b = int(inputs[index])
        index +=1
        color = (color_r, color_g, color_b)

        pos_x = float(inputs[index])
        index +=1
        pos_y = float(inputs[index])
        index +=1
        pos_z = float(inputs[index])
        index +=1

        position = numpy.array([pos_x * xyz_coord[0], pos_y * xyz_coord[1], pos_z * xyz_coord[2]])

        new_scene.addPointLight(position, color)

    # FIM DA LEITURA DOS INPUTS

    # checa se cam_forward e cam_up são aceitos
    if (cam_forward[0] == 0 and cam_forward[1] == 0 and cam_forward[2] == 0) or (cam_up[0] == 0 and cam_up[1] == 0 and cam_up[2] == 0):
        print('cam_forward e cam_up não podem ser [0,0,0] ou paralelas')
    else: # Render da imagem
        print('gerando imagem...')
        render(res_horizontal, res_vertical, size_pixel,cam_dist, cam_pos, cam_forward, cam_up, new_scene, max_depth)