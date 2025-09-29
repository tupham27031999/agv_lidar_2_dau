
from gtts import gTTS
import pygame
import os
import path
import threading
from support_main.lib_main import edit_csv_tab

path_phan_mem = path.path_phan_mem
path_folder_mp3 = path_phan_mem + "/mp3"
path_admin = path_phan_mem + "/setting/admin_window.csv"

if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"


data_admin = edit_csv_tab.load_all_stt(path_admin)
music_on = 0
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "music_on": # sai so cap nhat map
            music_on = int(float(data_admin[i][1]))
pygame.mixer.init()
def creat_thread():
    global stop_thread
    print("creat_thread")
    stop_thread = 0
threading_sound = threading.Thread(target=creat_thread)
stop_thread = 0
name_music = ""
name_music_old = ""
start_sound = 0
new_thread = 1
connect_sound = True
def disconnect_sound():
    global connect_sound
    pygame.mixer.music.stop()
    connect_sound = False
def creat_music(text, name, lang='vi'):
    temp_file = path_folder_mp3 + "/" + name + ".mp3"
    if os.path.exists(temp_file) == False:
        tts = gTTS(text=text, lang=lang)
        tts.save(temp_file)
# Hàm để phát âm thanh
def sound_speak():
    global name_music, start_sound, connect_sound, name_music_old
    while connect_sound and music_on == 1:
        temp_file = ""
        if name_music_old != name_music or pygame.mixer.music.get_busy() == False:
            name_music_old = name_music
            if name_music != "":
                temp_file = path_folder_mp3 + "/" + name_music + ".mp3"
                if os.path.exists(temp_file) == False:
                    temp_file = path_folder_mp3 + "/" + "music_3" + ".mp3"
            if os.path.exists(temp_file) == True:
                pygame.mixer.music.load(temp_file)
                pygame.mixer.music.play()
            else:
                pygame.mixer.music.stop()



# Hàm xử lý sự kiện bàn phím
def handle_key_event(event):
    global name_music
    if event.type == pygame.KEYDOWN:
        if event.key == pygame.K_1:
            name_music = "music_3"
        elif event.key == pygame.K_2:
            name_music = "re_phai"
        elif event.key == pygame.K_3:
            name_music = "re_trai"
        elif event.key == pygame.K_4:
            name_music = "lui_phai"
        elif event.key == pygame.K_5:
            name_music = "lui_trai"
        elif event.key == pygame.K_6:
            name_music = "co_vat_can"

# Hàm chính để chạy chương trình
def main():
    threading.Thread(target=sound_speak).start()
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("Key Event Handler")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            handle_key_event(event)
    disconnect_sound()
    pygame.quit()


# if __name__ == "__main__":
#     main()

# creat_music("車は右折しています", "re_phai_japan","ja")
# creat_music("車は左折しています", "re_trai_japan","ja")
# creat_music("車は右に後退している", "lui_phai_japan","ja")
# creat_music("車は左に後退している", "lui_trai_japan","ja")
# creat_music("障害がある", "vat_can_japan","ja")

# creat_music("xe đang rẽ phải", "re_phai_VN","vi")
# creat_music("xe đang rẽ trái", "re_trai_VN","vi")
# creat_music("xe đang lùi phải","lui_phai_VN","vi")
# creat_music("xe đang lùi trái", "lui_trai_VN","vi")
# creat_music("có vật cản", "vat_can_VN","vi")