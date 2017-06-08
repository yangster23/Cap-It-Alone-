#!/usr/bin/python
from django.core.management.base import BaseCommand, CommandError
from leap.models import User
import capItAlone

class Command(BaseCommand):
    help = 'runs the leap motion script'

    def add_arguments(self, parser):
        parser.add_argument('user_id', nargs='+', type=int)

    def handle(self, *args, **options):
        for user_id in options['user_id']:
            try:
                user = User.objects.get(pk=user_id)
            except User.DoesNotExist:
                raise CommandError('User "%s" does not exist' % user_id)

            os.system("capItAlone.py 1")
            user.save()

            self.stdout.write(self.style.SUCCESS('SUCCESSFULL')
