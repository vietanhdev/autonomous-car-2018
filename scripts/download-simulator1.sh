echo "Downloading Simulator 1"
mkdir -p tmp
cd tmp
../scripts/download-gdrive.sh "https://drive.google.com/uc?export=download&id=1eV_nAfa8jUwL8yAeRziy6F4P-j_UoaYn" "simulator1.zip"
cd ..

echo "Extracing ZIP file"
mkdir -p simulators
unzip tmp/simulator1.zip -d simulators/