import json
from io import BytesIO
import shutil
from dataclasses import asdict
from stair_for_bvbs.models import RebarforBVBS, RebarBVBS, RebarGeoBVBS
import numpy as np
from .models import WriteRebarJSON, ReadRebarJSON, Subs, GEO
import zipfile
import os


class MyEncode(json.JSONEncoder):
    def default(self, obj):
        print(type(obj))
        print(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(MyEncode, self).default(obj)


def create_json(rebar_for_BVBS: RebarforBVBS):
    file_names = []
    write_rebar_JSONs = []
    read_rebar_JSONs = []
    for attr, value in rebar_for_BVBS.__dict__.items():
        value: RebarBVBS
        geos = []
        for geo in value.geometric:
            geo: RebarGeoBVBS
            if geo.angle == 0:
                geos.append(
                    GEO(
                        length=geo.length,
                        angle=geo.angle,
                        lengthCompensation=0,
                        angleCompensation=0,
                        direction=False,
                        arc=False,
                        retract=False,
                    )
                )
            else:
                if geo.angle == 180:  # 测试  若是180度弯折  arc参数
                    geos.append(
                        GEO(
                            length=geo.length,
                            angle=geo.angle,
                            lengthCompensation=0,
                            angleCompensation=0,
                            direction=True,
                            arc=True,
                            retract=False,
                        )
                    )
                else:
                    geos.append(
                        GEO(
                            length=geo.length,
                            angle=geo.angle,
                            lengthCompensation=0,
                            angleCompensation=0,
                            direction=True,
                            arc=False,
                            retract=False,
                        )
                    )
        subs = Subs(geos=geos)
        write_rebar_JSON = WriteRebarJSON(
            billcode=value.mark,
            plan=value.rebar_quantity,
            finish=0,
            diameter=value.rebar_diameter,
            subs=subs,
        )
        read_rebar_JSON = ReadRebarJSON(
            billcode=value.mark,
            plan=value.rebar_quantity,
            finish=value.rebar_quantity,
            diameter=value.rebar_diameter,
            subs=subs,
        )
        file_names.append(attr)
        write_rebar_JSONs.append(write_rebar_JSON)
        read_rebar_JSONs.append(read_rebar_JSON)
    return file_names, write_rebar_JSONs, read_rebar_JSONs


def save_string_to_json_file(file_names, write_rebar_JSONs, read_rebar_JSONs):
    zip = zipfile.ZipFile("data/rebar_json.zip", "w", zipfile.ZIP_DEFLATED)
    for i in range(len(file_names)):
        with open(
            os.path.join("data/rebar_json", "write_{}.json".format(file_names[i])), "w"
        ) as f:
            str_in = json.dumps(asdict(write_rebar_JSONs[i]), cls=MyEncode)
            f.write(str_in)
            f.close()
            zip.write(f.name)
        with open(
            os.path.join("data/rebar_json", "read_{}.json".format(file_names[i])), "w"
        ) as f:
            str_in = json.dumps(asdict(read_rebar_JSONs[i]), cls=MyEncode)
            f.write(str_in)
            f.close()
            zip.write(f.name)
    shutil.rmtree("data/rebar_json")
    zip.close()
    print(f"rebar_json.zip saved successfully.")


def make_zip_content_export(file_names, write_rebar_json, read_rebar_json) -> bytes:
    """
    写入压缩文件,并以bytes 格式返回
    Args:
        file_names:
        write_rebar_json:
        read_rebar_json:

    Returns:
        contents: bytes
    """
    zip_content = BytesIO()

    with zipfile.ZipFile(zip_content, "w", zipfile.ZIP_DEFLATED) as zip_file:
        for i, file_name in enumerate(file_names):
            for j in range(2):
                zip_file.writestr(
                    f"{['write', 'read'][j]}_{file_name}.json",
                    json.dumps(
                        asdict([write_rebar_json, read_rebar_json][j][i]),
                        cls=MyEncode,
                    ),
                )
    zip_content.seek(0)
    return zip_content.read()
