import os
home = os.environ["HOME"]

launcher = ["[Desktop Entry]", f"Name=Project-L Main", f"Exec=cd /home/$(whoami)/Project-L && python3 main.py &", "Type=Application", "X-GNOME-Autostart-enabled=true"]
dr = home+"/.config/autostart/"
if not os.path.exists(dr):
    os.makedirs(dr)
file = dr+"project_l_main.desktop"

if not os.path.exists(file):
    with open(file, "wt") as out:
        for l in launcher:
            out.write(l+"\n")
    print("[Startup Script Creator] File created successfully")
else:
    print("[Startup Script Creator] File exists, choose another name")
