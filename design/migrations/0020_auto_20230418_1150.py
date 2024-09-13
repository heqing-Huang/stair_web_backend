# Generated by Django 3.2 on 2023-04-18 03:50

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ("design", "0019_fileexport_dxf"),
    ]

    operations = [
        migrations.AddField(
            model_name="modelconstructiondata",
            name="deflection_coefficient_1",
            field=models.FloatField(default=0, null=True, verbose_name="L0<7m"),
        ),
        migrations.AddField(
            model_name="modelconstructiondata",
            name="deflection_coefficient_2",
            field=models.FloatField(default=0, null=True, verbose_name="7m<=L0<9m"),
        ),
        migrations.AddField(
            model_name="modelconstructiondata",
            name="deflection_coefficient_3",
            field=models.FloatField(default=0, null=True, verbose_name="7m<=L0"),
        ),
    ]
