# RASPBERRY PI 4B PRESETS
Given that we will use a Raspberry Pi for Node Red implementation, as well as data post-processing, here's a guide on how to set up the device with all the software needed
## 1. Install Ubuntu
You will need: 
- An SD card
- A power chord for RPi
- A laptop
Steps:
1. Decide which Linux Distribution you want.
   In our case, we decided to use Ubuntu Server, since there was no need for a UI installation. 
2. Go to [Ubuntu's webpage](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#4-boot-ubuntu-server) and install the Raspberry Pi Imager for Windows
3. Once installed, choose your board and the OS you want. We chose 22.04.3 LTS because further versions required a higher RAM and were not LTS. 
![image](https://github.com/albacorreal/infind/assets/99867718/32c5c942-5194-4916-ba6e-1603bdfc8f97)
4. Insert your SD card in the computer and select it as Storage (WARNING: everything inside the SD card will be erased). Memory no lower than 64GB is recommended for this installation. We used a 128GB Kingston SD card for this purpose
5. Click on Next and wait for the process to take place
   Before starting, the Imager asks you if you want to edit specific settings for setup. I recommend using them, but I will explain how to do it from scratch for didactical purposes. 
7. Once finished, the Imager will require you to unmount the SD card. Do so and mount it again on the laptop before plugin the RPi.
8. Change network settings
   Open the File Manager and, inside the SD card, choose the file named "network-config".
   Open it in notepad and uncomment the wifis section. Set your wifi name between " ", and change the password. It should look like this:
   ~~~
   wifis:
     "My_Wifi_name":
       [...]
     password: "my_password"
   ~~~
9. Unmount the SD card and plug it in the Raspberry Pi. Let it think for around 2 minutes.
10. Search the Raspberry Pi's IP Address
    I searched for it inside my Router's settings. Just write "192.168.1.1" on any internet search bar and enter your router's password (usually in the back of your device).
    Once inside, look for a device named "ubuntu" and note down the IP adress. 
11. Final settings for Raspberry
    Open a PowerShell tab in your laptop
    Write the following command:
    ~~~
    ssh ubuntu@<raspberrypi_ip_address>
    ~~~
    It will ask you to enter the pasword, which is "ubuntu". Once entered, it will oblige you to change it immediately. PLEASE NOTE THE PASSWORD, otherwise you will not be able to access the RPi.
    Once you change the password, it will shut the connection down, so please re-enter the previous command for accessing the RPi.
12. Test the installation
    Write the following command:
    ~~~
    cat /etc/os-release
    ~~~
    It should show the Ubuntu version you installed.
## 2. Install NodeRed
1. Install with the command: 
~~~
bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)
~~~
2. limit the memory usage by entering: 
~~~
node-red-pi --max-old-space-size=256
~~~
3. Open NodeRed
   Write <raspberrypi_IP>:1880 in your search bar, it will automatically open NodeRed.

## 3. Install MongoDB
~~~
# Install the MongoDB 4.4 GPG key:
wget -qO - https://www.mongodb.org/static/pgp/server-4.4.asc | sudo apt-key add -

# Add the source location for the MongoDB packages:
echo "deb [ arch=amd64,arm64 ] https://repo.mongodb.org/apt/ubuntu focal/mongodb-org/4.4 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-4.4.list

# Download the package details for the MongoDB packages:
sudo apt-get update

# Install MongoDB:
sudo apt-get install -y mongodb-org
~~~

## 4. Install MQTT
1. Insert the following commands: 
~~~
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable mosquitto.service
mosquitto -v
~~~
2. Allow external unauthorised connections
   Change the config file with:
 ~~~
 sudo nano /etc/mosquitto/mosquitto.conf
 ~~~
  Add this at the end of the file: 
  ~~~
  listener 1883
  allow_anonymous true
  ~~~
3. Reboot Mosquitto
~~~
sudo systemctl restart mosquitto
~~~
