echo "Downloading Simulator 3"
mkdir -p tmp
cd tmp
../scripts/download-gdrive.sh "https://drive.google.com/uc?export=download&id=1Hxt7vPi9-ueL5NUsnJR16fcNxVrYTJc4" "simulator3.zip"
cd ..

echo "Extracing ZIP file"
mkdir -p simulators
unzip tmp/simulator3.zip -d simulators/