git init
git add .
git commit -m "Update"
git branch -M main
git remote add origin https://github.com/Magma0716/UWB-Program-DW3000.git
git push -u origin main

# safety version
git add .
git commit -m "Update"
git pull origin main
git push origin main