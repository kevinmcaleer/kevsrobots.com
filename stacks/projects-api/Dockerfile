FROM python:3.12-slim

WORKDIR /usr/src/app

RUN apt-get update && apt-get install -y curl ffmpeg fonts-dejavu-core && rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV PORT=8300
EXPOSE 8300

HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://127.0.0.1:8300/health || exit 1

CMD ["uvicorn", "projects_api.main:app", "--host", "0.0.0.0", "--port", "8300"]
