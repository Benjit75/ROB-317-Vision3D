import numpy as np
import cv2

print("Version d'OpenCV: ",cv2. __version__)
		
# Ouverture de l'image
PATH_IMG = './Images_Homographie/'

img = np.uint8(cv2.imread(PATH_IMG+"Pompei.jpg"))

(h,w,c) = img.shape
print("Dimension de l'image :",h,"lignes x",w,"colonnes x",c,"couleurs")

def select_points(event, x, y, flags, param):
	global points_selected,X_init
	global img,clone
	if (event == cv2.EVENT_FLAG_LBUTTON):
		x_select,y_select = x,y
		points_selected += 1
		cv2.circle(img,(x_select,y_select),8,(0,255,255),1)
		cv2.line(img,(x_select-8,y_select),(x_select+8,y_select),(0,255,0),1)
		cv2.line(img,(x_select,y_select-8),(x_select,y_select+8),(0,255,0),1)
		X_init.append( [x_select,y_select] )
	elif event == cv2.EVENT_FLAG_RBUTTON:
		points_selected = 0
		img = clone.copy()
		
clone = img.copy()
points_selected = 0
X_init = []
cv2.namedWindow("Image initiale")
cv2.setMouseCallback("Image initiale",select_points)

while True:
	cv2.imshow("Image initiale",img)
	key = cv2.waitKey(1) & 0xFF
	if (key == ord("q")) & (points_selected >= 4):
		break
		
# Conversion en array numpy
X_init = np.asarray(X_init,dtype = np.float32) 		
print("X_init =",X_init)
X_final = np.zeros((points_selected,2),np.float32)
for i in range(points_selected):
	string_input = "Correspondant de {} ? ".format(X_init[i])
	X_final[i] = input(string_input).split(" ",2)
print("X_final =",X_final)

# Votre code d'estimation de H ici
A = []
for i in range(points_selected):
    x, y = X_init[i][0], X_init[i][1]
    xp, yp = X_final[i][0], X_final[i][1]
    A.append([-x, -y, -1, 0, 0, 0, x*xp, y*xp, xp])
    A.append([0, 0, 0, -x, -y, -1, x*yp, y*yp, yp])

A = np.array(A)
U, S, Vh = np.linalg.svd(A)
H = Vh[-1].reshape(3, 3)
H = H / H[2, 2]  # Normalisation
print("Matrice d'homographie H :\n", H)

# Affichage de l'image rectifiée
img_warp = cv2.warpPerspective(clone, H, (w,h))
cv2.imshow("Image rectifiee",img_warp)
k = cv2.waitKey(0)
if (k == ord("q")):
	cv2.destroyAllWindows()
elif (k == ord("s")):
	cv2.imwrite("img_rectified.png",img_warp)
	cv2.destroyAllWindows()

