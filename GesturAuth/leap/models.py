# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models
from django.utils.encoding import python_2_unicode_compatible
# Create your models here.

@python_2_unicode_compatible  # only if you need to support Python 2
class User(models.Model):
    username = models.CharField(max_length=100)

    def __str__(self):
        return self.username

    class Meta:
        ordering = ['id']
