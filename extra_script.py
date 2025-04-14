# extra_script.py
from SCons.Script import DefaultEnvironment

# Importiere die Build-Umgebung
env = DefaultEnvironment()

# Definiere eine Funktion, die vor dem Firmware-Upload ausgeführt wird
def before_upload(source, target, env):
    print("Attempting to upload filesystem image...")
    # Führe den PlatformIO-Befehl zum Hochladen des Dateisystems aus
    # Stellt sicher, dass es im Kontext der aktuellen Umgebung läuft
    #env.Execute("$PYTHONEXE $PROJECT_CORE_DIR/packages/tool-esptoolpy/esptool.py --chip esp32 --port $UPLOAD_PORT --baud $UPLOAD_SPEED write_flash --flash_size detect 0x10000 $BUILD_DIR/littlefs.bin")
    # Alternative (einfacher, aber evtl. weniger robust bei Pfadproblemen):
    env.Execute("pio run -t uploadfs")
    print("Filesystem image upload command executed.")

# Registriere die Funktion, die *vor* dem 'upload'-Target ausgeführt werden soll
env.AddPreAction("upload", before_upload)

# Optional: Du könntest auch AddPostAction verwenden, um es *nach* dem Upload zu tun,
# aber vorher ist meist sinnvoller, damit die Firmware die Dateien direkt findet.
