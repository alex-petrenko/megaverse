PYTHON ?= python

export REPO=voxel-rl
export BASE_TAG=$(shell ${PYTHON} -c 'import hashlib; sha = hashlib.sha1((open("docker/Dockerfile.base").read() + open("requirements/requirements.txt").read()).encode()); print(sha.hexdigest())')
BRANCH = $(shell git rev-parse --abbrev-ref HEAD)
VERSION = $(shell git rev-parse --short HEAD)
DATE = $(shell date +%F)
export TAG=$(BRANCH)-$(VERSION)-$(DATE)

.PHONY: pull build up push shell docker-bash

pull-%:
	docker-compose pull $(subst pull-,,$@)

build-%:
	docker-compose build $(subst build-,,$@)

up-%:
	docker-compose up -d $(subst up-,,$@)

push-%:
	docker-compose push $(subst push-,,$@)

shell:
	@echo TAG=$(TAG) BASE_TAG=$(BASE_TAG)

docker-bash:
	docker-compose run --it --rm dev bash

.PHONY: upload upload-test clean

upload:
	python3 -m twine upload --repository pypi dist/*

upload-test:
	python3 -m twine upload --repository testpypi dist/*

clean:
	rm -rf build dist

