from cmath import atan, sqrt
from http.server import SimpleHTTPRequestHandler
from sys import set_coroutine_origin_tracking_depth
import numpy
from PIL import Image
from pkg_resources import split_sections

# classe para passar os dados quando houver algum hit
class rayhit:
    def __init__(self, hitObj,hitPoint, hitNormal, hitDistance, color):
        self.hitObj = hitObj
        self.hitPoint = hitPoint
        self.hitNormal = hitNormal
        self.hitDistance = hitDistance
        self.color = color

# classe principal dos objetos da cena
class scene_object:
    def __init__(self, position = [0,0,0], color = (255,0,0)):
        self.position = position
        #self.radius = radius
        self.color = color
    
    # retorna a normal no ponto p
    def getNormal(self, p):
        return
    
    # retorna 0 se não houver hit, se houver retorna um rayhit
    def intersection(self, origin, direction):
        return

# calsse do objeto: plano
class plane(scene_object):
    def __init__(self, position=[0, 0, 0], normal=[0,1,0], color=(255, 0, 0)):
        self.normal = normalized(normal)
        super().__init__(position, color)
    
    def getNormal(self, p):
        return self.normal
    
    def intersection(self, origin, direction):
        
        ldotn = numpy.dot(normalized(direction),normalized(self.normal))
        if abs(ldotn) <= 0.001:
            return 0
        
        t = numpy.dot((self.position - origin), self.normal) / ldotn
        
        # erro quando a camera está na area do plano

        if t<0:
            return 0
        else:
            hitPoint = origin + direction * t
            normal = self.getNormal(hitPoint)
            color = self.color
            return rayhit(self, hitPoint, normal, t, color)


# classe do objeto: esfera
class sphere(scene_object):

    def __init__(self, position=[0, 0, 0], radius=1, color = (255,0,0)):
        self.radius = radius
        super().__init__(position, color)
    
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
            # teste de sombreamento simples
            #ndotl = numpy.dot(normal, -normalized(numpy.array([1,-2,.5])))
            #if ndotl<0.2:
            #    ndotl = .2
            #color = (int(color[0] * ndotl), int(color[1] * ndotl), int(color[2] * ndotl))

            return rayhit(self, hitPoint, normal, hitDist, color)

# classe da cena, vai guardar os objetos
class scene_main:
    def __init__(self):
        
        # criação de objetos para popular a cena
        self.objs = []
        self.bg_color = (0,0,0)

        #self.objs.append(sphere([-4,-5,12], 5, (0,255,255)))
        #self.objs.append(sphere([0,0,0],0.6))
        #self.objs.append(sphere([2,1,0.3],0.5, (0,255,0)))
        #self.objs.append(sphere([2,1,7], 2, (0,0,255)))
        #self.objs.append(plane([0,-.25,0],[0,1,0] , color= (150,150,150)))
        #self.objs.append(plane([3.5,0,0],[-1,-1,0] , color= (150, 0,150)))
    
    def setBackground_Color(self, color):
        self.bg_color = color
    
    def getBackground_Color(self):
        return self.bg_color

    def addSphere(self, position, radius, color):
        self.objs.append(sphere(position, radius, color))
    
    def addPlane(self, position, normal, color):
        self.objs.append(plane(position, normal, color))


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
    color = scene.getBackground_Color()
    hit = trace(origin,direction,scene)
    if hit:
        color = hit.color
    
    return color

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

# funcao que retorna um vetor normalizado
def normalized(vec):
    n = numpy.linalg.norm(vec)
    if n ==0:
        return vec
    else:
        return vec / n


# valores padrão
# para mudara  resolucao sem alterar o fov, quanto maior melhor a imagem e mais lento fica
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

leituraArquivo = input('Ler do arquivo "inputs"? (s para sim): ')

if leituraArquivo == 's':
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

    cam_pos = numpy.array([cam_pos_x, cam_pos_y, cam_pos_z])
    cam_forward = numpy.array([cam_forward_x, cam_forward_y, cam_forward_z]) - cam_pos
    cam_up = numpy.array([cam_up_x, cam_up_y, cam_up_z])

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

        obj_select = inputs[index]
        index +=1

        pos_x = float(inputs[index])
        index +=1
        pos_y = float(inputs[index])
        index +=1
        pos_z = float(inputs[index])
        index +=1

        position = numpy.array([pos_x, pos_y, pos_z])

        if obj_select == '*':
            radius = float(inputs[index])
            index +=1

            new_scene.addSphere(position, radius, color)
        else:
            normal_x = float(inputs[index])
            index +=1
            normal_y = float(inputs[index])
            index +=1
            normal_z = float(inputs[index])
            index +=1

            normal = normalized([normal_x, normal_y, normal_z])

            new_scene.addPlane(position, normal, color)
else:
    res_vertical = int(input("resolucao vertical"))
    res_horizontal = int(input("resolucao horizontal"))
    size_pixel = float(input("tamanho do pixel"))
    cam_dist = float(input("distancia camera"))
    cam_pos_x = float(input("camera pos x"))
    cam_pos_y = float(input("camera pos y"))
    cam_pos_z = float(input("camera pos z"))
    cam_forward_x = float(input("camera mira x"))
    cam_forward_y = float(input("camera mira y"))
    cam_forward_z = float(input("camera mira z"))
    cam_up_x = float(input("camera up x"))
    cam_up_y = float(input("camera up y"))
    cam_up_z = float(input("camera up z"))
    bg_color_r = int(input("cor fundo r (0 a 255)"))
    bg_color_g = int(input("cor fundo g (0 a 255)"))
    bg_color_b = int(input("cor fundo b (0 a 255)"))
    k_obj = int(input("quantidade de objetos"))

    new_scene.setBackground_Color((bg_color_r,bg_color_g,bg_color_b))

    cam_pos = numpy.array([cam_pos_x, cam_pos_y, cam_pos_z])
    cam_forward = numpy.array([cam_forward_x, cam_forward_y, cam_forward_z]) - cam_pos
    cam_up = numpy.array([cam_up_x, cam_up_y, cam_up_z])

    cam_forward = normalized(cam_forward)
    cam_up = normalized(cam_up - numpy.dot(cam_forward, cam_up) * cam_forward)

    for i in range(k_obj):
        color_r = int(input("color r"))
        color_g = int(input("color g"))
        color_b = int(input("color b"))
        color = (color_r, color_g, color_b)

        obj_select = input("* para esfera / para plano")
        
        pos_x = float(input("pos x"))
        pos_y = float(input("pos y"))
        pos_z = float(input("pos z"))
        
        position = numpy.array([pos_x, pos_y, pos_z])

        if obj_select == '*':
            radius = float(input("radius"))

            new_scene.addSphere(position, radius, color)
        else:
            normal_x = float(input("normal x"))
            normal_y = float(input("normal y"))
            normal_z = float(input("normal z"))

            normal = normalized([normal_x, normal_y, normal_z])

            new_scene.addPlane(position, normal, color)


# checa se cam_forward e cam_up são aceitos
if (cam_forward[0] == 0 and cam_forward[1] == 0 and cam_forward[2] == 0) or (cam_up[0] == 0 and cam_up[1] == 0 and cam_up[2] == 0):
    print('cam_forward e cam_up não podem ser [0,0,0] ou paralelas')
else:
    print('gerando imagem...')
    render(res_horizontal, res_vertical, size_pixel,cam_dist, cam_pos, cam_forward, cam_up, new_scene)