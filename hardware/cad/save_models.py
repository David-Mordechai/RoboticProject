# FreeCAD script: save the open Robot and BoardStack documents to out/ as .FCStd + .step (no .FCBak backups).
#   python fc.py run save_models.py
import os
import FreeCAD as App
import Import

App.ParamGet("User parameter:BaseApp/Preferences/Document").SetBool("CreateBackupFiles", False)
out = r"C:\Development\RoboticProject\hardware\cad\out"
os.makedirs(out, exist_ok=True)
for name in ("Robot", "BoardStack"):
    if name not in App.listDocuments():
        continue
    d = App.getDocument(name)
    d.saveAs(os.path.join(out, name + ".FCStd"))
    Import.export([o for o in d.Objects if hasattr(o, "Shape")], os.path.join(out, name + ".step"))
print(sorted(os.listdir(out)))
