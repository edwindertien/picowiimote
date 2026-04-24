# upload_fs_pre.py — runs before firmware upload
# Automatically uploads LittleFS filesystem image (data/mapping.json)
# so you only need: pio run -t upload
Import("env")
env.AddPreAction("upload", env.VerboseAction(
    lambda *args, **kwargs: env.Execute("pio run -t uploadfs"),
    "Uploading filesystem (data/mapping.json)..."
))