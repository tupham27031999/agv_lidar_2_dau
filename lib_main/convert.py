import numpy as np
import os,cv2
from tkinter.messagebox import showerror, showwarning, showinfo
from PIL import Image, ImageTk


def img_resize_vid(video, resize_x, resize_y):
    img_rs = cv2.resize(video, (int(resize_x),int(resize_y)))
    img_rs = cv2.cvtColor(img_rs, cv2.COLOR_BGR2RGB)
    img_rs = ImageTk.PhotoImage(Image.fromarray(img_rs))
    return img_rs
def img_resize_path(path, resize_x, resize_y):
    img_rs = cv2.imread(path)
    img_rs = cv2.resize(img_rs, (int(resize_x),int(resize_y)))
    img_rs = cv2.cvtColor(img_rs, cv2.COLOR_BGR2RGB)
    img_rs = ImageTk.PhotoImage(Image.fromarray(img_rs))
    return img_rs

def resize_img(input,number):
    img1 = input
    if len(img1.shape) == 2:
        h0,w0 = img1.shape
    else:
        h0,w0,_ = img1.shape
    # ty le resize
    if w0 >= number:
        resize_number = w0/number
        image = cv2.resize(img1.copy(),(int(w0/resize_number),int(h0/resize_number)))
    else:
        resize_number = 1
        image = img1.copy()
    return image,resize_number
def resize_img_shape(input,number):
    img1 = input
    if len(img1.shape) == 2:
        h0,w0 = img1.shape
    else:
        h0,w0,_ = img1.shape
    resize_number = w0/number
    image = cv2.resize(img1.copy(),(int(w0/resize_number),int(h0/resize_number)))
    return image,resize_number

def show_error(name_error):
    showerror(title='Error',message=str(name_error))
    return 1
def show_warning(name_warning):
    showwarning(title='Error',message=str(name_warning))
    return 1
def list_atwork(link_folder,showerror_list_atwork):
    list_at = []
    if os.path.exists(link_folder) == True:
        try:
            ds = os.listdir(link_folder)
            for i in range(0,len(ds)):
                list_at.append(link_folder + "/" + ds[i])
        except:
            pass
    else:
        if showerror_list_atwork == 0:  
            showerror_list_atwork = show_error("Không tồn tại đường link " + str(link_folder))
    return  list_at,showerror_list_atwork


