# Задания полуфинала олимпиады "Я - профессионал" 2022-2023 по робототехнике - Магистратура
[![Telegram](https://img.shields.io/badge/Telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white)](https://t.me/iprofirobots) [![Yandex IProfi](https://img.shields.io/badge/yandex-%23FF0000.svg?&style=for-the-badge&logo=yandex&logoColor=white)](https://yandex.ru/profi/profile/?page=contests) [![Mail](https://custom-icon-badges.demolab.com/badge/-iprofi.robotics@yandex.ru-red?style=for-the-badge&logo=mention&logoColor=white)](mailto:iprofi.robotics@yandex.ru)


![scene pic](docs/figures/scene_view.png)


Репозиторий содержит ROS-пакет с минимальным *решением* задачи. Участнику следует, модифицируя этот пакет, решить задачу.

## Задача

Дано складское помещение, разделенное на три зоны: разгрузки, сортировки и накопления.

В зоне разгрузки функционирует манипуляционный робот на подвижном потолочном подвесе с двумя степенями свободы, способный перемещаться вдоль этой зоны.

В зоне сортировки и накопления доступны несколько мобильных платформ; навигационная цветная разметка, нанесенная на поверхность пола; и некоторое количество статических препятствий.

Участникам предлагается с использованием доступных роботов, реализовать алгоритм управления складом, который позволит переместить наибольшее число мешков картошки из зоны разгрузки в обозначенные цветами (красным, оранжевым, желтым, зеленым, синим) зоны накопления. Каждый мешок должен быть доставлен в соответствующую цветовой маркировке мешка зону.

В закрытых тестовых сценариях могут быть изменены конфигурация статических препятствий и порядок цветовых зон и линий навигации.

Требуется разработать техническое решение, включая алгоритмическое и программное обеспечения системы управления и обработки сенсорной информации, в форме программного пакета для ROS на языках программирования С++ и/или Python.


## Как все работает

Для решения задачи доступны два read-only docker-образа:

- [base] `registry.gitlab.com/beerlab/iprofi2023/problem/master_problem_2023/base:ubuntu-latest` -- включает все зависимости.

- [scene] `registry.gitlab.com/beerlab/iprofi2023/problem/master_problem_2023/scene:ubuntu-latest` -- собран на базе предыдущего и дополнительно включает файлы сцены в gazebo.

Запуск включает два шага:
- В контейнере сервиса `scene` на основе образа `[scene]` запускается сцена в симуляторе gazebo [master_problem_scene](https://gitlab.com/beerlab/iprofi2023_dev/problem/master_scene).
- В контейнере сервиса `problem` на основе образа `[base]` запускается решение [master_problem_2023](https://gitlab.com/beerlab/iprofi2023/problem/master_problem_2023).

Для автоматизации запуска запуска docker-контейнеров используется инструмент docker compose. Описание параметров запуска доступно в: `docker-compose.yml `.

*Note! Если вы используется систему с GPU от Nvidia, доступны версии образов с тегом `nvidia-latest` и `docker-compose.nvidia.yml`x*


## Установка и настройка окружения

Для настройки окружения необходимо иметь одну из перечисленных операционных систем:
1. Ubuntu 18.04 и старше
2. Windows 10 и старше, с установленным WSL (Не рекомендуется).

Для подготовки окружения необходимо сделать следующее:
1. Установить docker-engine: [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).  
2. Также необходимо установить docker-compose-plugin: [Docker Compose](https://docs.docker.com/compose/install/linux/).  
3. Если вы планируете использовать видеокарту, установите также nviidia-container-toolkit: [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
4. Добавить в группу docker пользователя

    ```bash
    sudo groupadd docker 
    sudo usermod -aG docker $USER 
    newgrp docker
    ```


## Как запустить начальное(базовое решение)
**Сделать форк репозитория** в корень gitlab своего юзера, **не изменяя имени репозитория**.  
Склонировать репозиторий:

```bash
git clone https://gitlab.com/<YOUR_NAME>/master_problem_2023.git
cd master_problem_2023
```

Дать права для подключения пользователю root к дисплею хоста:

```
xhost +local:docker
```

Запустить сцену и ros-пакет из этого репозитория:

```bash
docker compose -f docker-compose.yml up --build --pull always
```
*Note!* В файле `docker-compose.yml` хранится описание параметров запуска сцены и решения. По умолчанию запускается `example_node`

```bash
rosrun master_problem_2023 example_node
```

### Редактирование базового решения
Для редактирования доступны все файлы в репозтории, за исключение файлов `docker-compose*.yml`.  
Чтобы начать решать задание вы можете отредактировать файл `start.launch` выбрав запуск python или C++ версии программы решения. 

Если вы пишете на python, нужно, чтобы в `start.launch` была раскомментирована строка: 

    <node name="example_node" pkg="master" type="example.py" output="screen"></node>

Если вы пишете на C++, нужно, чтобы в `start.launch` была раскомментирована строка: 

    <node name="example_node" pkg="master" type="example_node" output="screen"></node>

## Дополнительные полезные команды

В случае необходимости пересборки используйте флаг `--build`:

    docker compose -f docker-compose.yml up --build

Для получения последней версии сцены (обновления) используейте флаг `--pull always`:

    docker compose -f docker-compose.yml up --build --pull always

### Подключение в контейнер

Для открытия новой bash-сессии в сервисе решения: `problem` используйте команду:

    docker compose exec problem bash

Для открытия новой bash-сессии в сервисе сцены: `scene` используйте команду:

    docker compose exec scene bash

### Рестарт сцены или решения по отдельности
Для перезапуска **решения** используйте:

    docker compose restart problem

Для перезапуска **сцены** используйте:

    docker compose restart scene


## Оценка

Оценивается количество перемещенных мешков с картошкой из зоны разгрузки в зону накопления. Каждый мешок должен быть перемещен в цветную зону накопления, соответствующую нанесенной на мешок цветной метке - такой сценарий перемещения считается успешным. **Один робот вмещает в себя только один мешок**.

- 0.5 балл – за каждый мешок, погруженный на мобильную платформу
- 0.5 балл – за каждый доставленный мешок в верную зону накопления
- 0.5 балл штрафа – за каждый перемещенный мешок в ошибочную зону накопления 
- 0.1 балл штрафа – за каждое столкновение с препятствиями

Время, которое дается роботу на выполнение задания **10 минут**.
