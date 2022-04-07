from cmath import atan, sqrt
import numpy
from PIL import Image

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
        self.objs.append(sphere([-4,-5,12], 5, (0,255,255)))
        self.objs.append(sphere([0,0,0],0.6))
        self.objs.append(sphere([2,1,0.3],0.5, (0,255,0)))
        self.objs.append(sphere([2,1,7], 2, (0,0,255)))
        self.objs.append(plane([0,-.25,0],[0,1,0] , color= (150,150,150)))
        self.objs.append(plane([3.5,0,0],[-1,-1,0] , color= (150, 0,150)))

# função principal para criar a imagem test.png com o resultado
def render(res_h, res_v, pxl_size,d,cam_pos,cam_forward,cam_up):
    # cria a cena
    new_scene = scene_main()

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
            img.putpixel((x,y), cast(cam_pos,ray_dir,new_scene))
     
    img.save('test.png')
    print("imagem salva")

def cast(origin, direction,scene):
    color = (0,0,0)
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

cam_forward = normalized(cam_forward)
cam_up = normalized(cam_up - numpy.dot(cam_forward, cam_up) * cam_forward)

# para conferir o field of view da camera, usar valor em torno de 90 para menos distorção
# fov = atan((0.05 * 300 * .5) / 7.5) * 57.2958 * 2
# print(fov)

# checa se cam_forward e cam_up são aceitos
if (cam_forward[0] == 0 and cam_forward[1] == 0 and cam_forward[2] == 0) or (cam_up[0] == 0 and cam_up[1] == 0 and cam_up[2] == 0):
    print('cam_forward e cam_up não podem ser [0,0,0] ou paralelas')
else:
    render(res_horizontal, res_vertical, size_pixel,cam_dist, cam_pos, cam_forward, cam_up)


