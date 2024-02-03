# Build your Docker image
docker build -t http://192.168.2.1:5000/search:latest .

# Push your Docker image
# docker push search:latest
docker push http://192.168.2.1:5000/search:latest