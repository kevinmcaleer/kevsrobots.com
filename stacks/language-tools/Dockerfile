FROM python:3.12-slim

WORKDIR /usr/src/app

RUN apt-get update && apt-get install -y --no-install-recommends curl && rm -rf /var/lib/apt/lists/*

# WordNet 3.1 for Look Up and Synonyms, from the wordnet-db npm package (the
# same copy markdown-studio uses). Only its dict/ files are kept (~30 MB).
ARG WORDNET_DB_VERSION=3.1.14
RUN mkdir -p /usr/share/wordnet \
    && curl -fsSL "https://registry.npmjs.org/wordnet-db/-/wordnet-db-${WORDNET_DB_VERSION}.tgz" \
       | tar -xz -C /usr/share/wordnet --strip-components=2 package/dict \
    && ls /usr/share/wordnet/index.noun

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY language_tools ./language_tools

ENV PORT=8310 \
    WORDNET_DIR=/usr/share/wordnet
EXPOSE 8310

HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://127.0.0.1:8310/health || exit 1

CMD ["uvicorn", "language_tools.main:app", "--host", "0.0.0.0", "--port", "8310"]
