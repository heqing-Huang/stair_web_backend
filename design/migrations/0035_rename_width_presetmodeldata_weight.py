# Generated by Django 4.2.16 on 2024-09-20 11:23

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ("design", "0034_rename_weight_presetmodeldata_width"),
    ]

    operations = [
        migrations.RenameField(
            model_name="presetmodeldata",
            old_name="width",
            new_name="weight",
        ),
    ]
