from cmath import atan, sqrt
import numpy
from PIL import Image

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
    def __init__(self, position = [0,0,0], color = (255,0,0), ka=1, kd=1, ks=1, phongN=1):
        self.position = position
        #self.radius = radius
        self.color = color
        
        self.ka = ka
        self.kd = kd
        self.ks = ks
        self.phongN = phongN
    
    # retorna a normal no ponto p
    def getNormal(self, p):
        return
    
    # retorna 0 se não houver hit, se houver retorna um rayhit
    def intersection(self, origin, direction):
        return

# classe do objeto: plano
class plane(scene_object):
    def __init__(self, position = [0,0,0], normal = [0,1,0], color = (255,0,0), ka=1, kd=1, ks=1, phongN=1):
        self.normal = normalized(normal)
        super().__init__(position, color, ka, kd, ks, phongN)
    
    def getNormal(self, p):
        return self.normal
    
    def intersection(self, origin, direction):
        #formula para interseção demonstrada no stratchapixel.com        
        ldotn = numpy.dot(normalized(direction),normalized(self.normal))
        if abs(ldotn) <= 0.000:
            return 0
        
        t = numpy.dot((self.position - origin), self.normal) / ldotn
        
        # ocorre um erro quando a posição da camera é faz parte do plano, o fica 0

        if t<0:
            return 0
        else:
            hitPoint = origin + direction * t
            normal = self.getNormal(hitPoint)
            color = self.color
            return rayhit(self, hitPoint, normal, t, color, direction)


# classe do objeto: esfera
class sphere(scene_object):

    def __init__(self, position = [0,0,0], radius = 1, color = (255,0,0), ka=1, kd=1, ks=1, phongN=1):
        self.radius = radius
        super().__init__(position, color, ka, kd, ks, phongN)
    
    def getNormal(self, p):
        return normalized(p - self.position)
    
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
            hitDist = tca - thc
            hitDist2 = tca + thc
            if hitDist2 < hitDist:
                hitDist = hitDist2
            hitPoint = origin + direction * hitDist

            normal = self.getNormal(hitPoint)

            color = self.color
            
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

    def addSphere(self, position, radius, color, ka, kd, ks, phongN):
        self.objs.append(sphere(position, radius, color, ka, kd, ks, phongN))
    
    def addPlane(self, position, normal, color, ka, kd, ks, phongN):
        self.objs.append(plane(position, normal, color, ka, kd, ks, phongN))

    def addPointLight(self, position, color):
        self.lights.append(pointLight(position, color))
    
    def setAmbientLight(self, color):
        self.ambientLight = color

class light:
    def __init__(self, position, color):
        self.position = position
        self.color = color

class pointLight(light):

    def __init__(self, position, color):
        super().__init__(position, color)

# função principal para criar a imagem test.png com o resultado
def render(res_h, res_v, pxl_size,d,cam_pos,cam_forward,cam_up, scene):
    # cria um Image todo preto
    img = Image.new('RGB', (res_h,res_v), color = (0,0,0))
    
    # acha o vetor da camera que aponta para a sua direita
    cam_right = numpy.asarray(numpy.cross(cam_up, cam_forward))
    
    # centro do pixel mais a esquerda e acima
    topleft = cam_pos + cam_forward * d + (cam_up *  (res_v - 1) - cam_right * (res_h - 1)) * pxl_size * 0.5
    
    # dispara o raio no centro de cada pixel e guarda sua cor na img
    for x in range(res_h):
        for y in range(res_v):
            ray_dir = normalized((topleft + (cam_up * -y + cam_right * x) * pxl_size) - cam_pos)
            img.putpixel((x,y), cast(cam_pos,ray_dir, scene))
     
    img.save('test.png')
    print("imagem salva")

def cast(origin, direction,scene):
    color = colorNormalize(scene.getBackground_Color())
    hit = trace(origin,direction,scene)
    if hit:
        color = shade(hit, scene)
    
    return colorDenormalize(color)

def trace(origin, direction, scene:scene_main):

    hit = 0
    for i in range(len(scene.objs)):
        checkHit = scene.objs[i].intersection(origin,direction)

        if checkHit:
            if hit:
                if checkHit.hitDistance < hit.hitDistance:
                    hit = checkHit
            else:
                hit = checkHit

    
    return hit

def shade(hit:rayhit, scene:scene_main):
    color_difuse = colorNormalize(hit.color)
    # cor é iniciada como a ambiente
    color =  colorScale(colorMul(color_difuse, colorNormalize(scene.ambientLight)), hit.hitObj.ka)

    for light in scene.lights:
        color_light = colorNormalize(light.color)
        l = light.position - hit.hitPoint
        lDist = numpy.linalg.norm(l)
        l = normalized(l)

        ndotl = numpy.dot(hit.hitNormal, l).real
        if ndotl > 0:
            # checar se tem algum objeto obstruindo a luz
            shadowHit = trace(hit.hitPoint + l *0.00001, l, scene)
            if shadowHit !=0 and shadowHit.hitDistance < lDist:
                continue
            
            # soma da cor difusa
            color = colorSum(color, colorScale(colorMul(color_light, color_difuse), ndotl * hit.hitObj.kd))
            
            rj = 2 * ndotl * hit.hitNormal - l

            view = normalized(-hit.ray)
            rjdotview = numpy.dot(rj,view).real
            if rjdotview < 0:
                rjdotview = 0
                
            # soma da parte specular
            color = colorSum(color, colorScale(color_light , hit.hitObj.ks * numpy.power(rjdotview, hit.hitObj.phongN)))

    return color

# funcao que retorna um vetor normalizado
def normalized(vec):
    n = numpy.linalg.norm(vec)
    if n ==0:
        return vec
    else:
        return vec / n

# multiplica cores
def colorMul(color1, color2):
    r1 = color1[0]
    g1 = color1[1]
    b1 = color1[2]
    
    r2 = color2[0]
    g2 = color2[1]
    b2 = color2[2]

    return (r1 * r2, g1 * g2, b1 * b2)

# multiplica cor por um escalar
def colorScale(color, f):
    return (color[0] * f, color[1] * f, color[2] * f)

def colorSum(color1, color2):
    r1 = color1[0]
    g1 = color1[1]
    b1 = color1[2]
    
    r2 = color2[0]
    g2 = color2[1]
    b2 = color2[2]

    return (r1+r2, g1+g2, b1+b2)

def colorNormalize(color):
    return (float(color[0]) / 255.0, float(color[1]) / 255.0, float(color[2]) / 255.0)

def colorDenormalize(color):
    f = max(1, *color)
    return (int(color[0] * 255.0/f), int(color[1] * 255.0/f), int(color[2] * 255.0/f))

# valores padrão
# para mudar a resolucao sem alterar o fov, quanto maior melhor a imagem e mais lento fica
res_factor = 1

res_horizontal = 300 * res_factor
res_vertical = 200 * res_factor
size_pixel = 0.05 / res_factor
cam_dist = 7.5
cam_pos = numpy.array([0,1,-5])
# se certificar de que cam_forward e cam_up não são paralelos o [0,0,0]
cam_forward = numpy.array([0,0,1])
cam_up = numpy.array([0,1,0])

# para conferir o field of view da camera, usar valor em torno de 90 para menos distorção
# fov = atan((0.05 * 300 * .5) / 7.5) * 57.2958 * 2
# print(fov)

new_scene = scene_main()
bg_color = (0,0,0)

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

        new_scene.addSphere(position, radius, color, ka, kd, ks, phongN)
    else:
        normal_x = float(inputs[index])
        index +=1
        normal_y = float(inputs[index])
        index +=1
        normal_z = float(inputs[index])
        index +=1

        normal = normalized([normal_x * xyz_coord[0], normal_y * xyz_coord[1], normal_z * xyz_coord[2]])

        new_scene.addPlane(position, normal, color, ka, kd, ks, phongN)

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


# checa se cam_forward e cam_up são aceitos
if (cam_forward[0] == 0 and cam_forward[1] == 0 and cam_forward[2] == 0) or (cam_up[0] == 0 and cam_up[1] == 0 and cam_up[2] == 0):
    print('cam_forward e cam_up não podem ser [0,0,0] ou paralelas')
else: # Render da imagem
    print('gerando imagem...')
    render(res_horizontal, res_vertical, size_pixel,cam_dist, cam_pos, cam_forward, cam_up, new_scene)