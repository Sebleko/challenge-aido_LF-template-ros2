
build:
	dts build_utils aido-container-build --use-branch daffy --ignore-untagged --force-login --push




submit:
	dts challenges submit


submit-bea:
	dts challenges submit --impersonate 1639 --challenge 'aido-LF*' --retire-same-label
