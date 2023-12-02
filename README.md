# assistive-shoulder-firmware

# Getting Started
- Follow the steps on [esp-idf](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c6/get-started/linux-macos-setup.html) website and install the latest version of esp-idf
- This project uses the LTS version 5.1(5.1.2 to be specific) which wil be supported till December 2025.
- Please ensure that you have the alias set in your bash fish or zsh path
```bash
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```
This will help you easily source esp idf whenever you want. 
> It is necessary to source esp idf whenever you open new terminal.Keep that in mind.

# Important commands
This project uses esp32-c6 so it is necessary to specify the target before building with following command
```bash
idf.py set-target esp32c6
```
To build the project just run 
```bash
idf.py build
```
If you are using linux it is necessary to provide correct access to non sudo user for that run this command
```bash
sudo chmod a+rw /dev/ttyACM0
```
To flash the chip run
```bash
idf.py flash -p /dev/ttyACM0
```
for port ttyACM0

# Good Practice
The master branch is protected and requires all commits to be signed before merging.
To sign commits follow this [article](https://docs.github.com/en/authentication/managing-commit-signature-verification/adding-a-gpg-key-to-your-github-account) by github.

# Compliance
> This project uses 2012 MISRA C Compliance all pull requests must follow it 
> Document whenever necessary.

