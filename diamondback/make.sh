#!/bin/bash
SOURCE_PATH=~/.bashrc

source $SOURCE_PATH

#bash make.sh -t all --cmd clean
#bash make.sh -t all --cmd strip-svn
#bash make.sh -t target1 target2 -c cmd opt1 opt2
#bash make.sh -t xsens_node --cmd cd --cmd make -j5
#bash make.sh -t seabee3_driver_base seabee3_driver --cmd rosmake

mode=0
n=0
num_commands=0
rel_target=""
abs_target=""

parseTargets()
{
	echo "parseTargets"
	echo "$@"
	n=0
	while [ "$1" != "" ] && [ "$1" != "--foreach" ] && [ "$1" != "--command" ] && [ "$1" != "--target" ]; do
		shift
		targets[$n]=$1
		n=$(($n + 1))
	done
}

parseCommands()
{
	shift
	echo "parseCommands"
	echo ">$commands_raw[1]"
	commands="$@"
}

findRosTarget()
{
#parse path into folders
#	while [  ]; do
#		
#	done
#	if [ ($(bash -c ls | grep manifest.xml)) == "manifest.xml" ]; then
#		offset=`expr index $HTML $OK`
#		token_len=`expr length $OK`
#	fi
	rel_target=$abs_target
}

usage()
{
	echo ""
	echo "Usage: bash make.sh [--foreach] --command \"cmd1 cmd2 ..\" --target t1 t2 .."
	echo ""
	echo "Note on --command: @ -> list of all targets; $ -> Nth target"
	echo ""
	echo "Example1: bash make.sh --foreach --command \"cd $ && make -j5 && cd ..\" --target xsens_node seabee3_driver"
	echo "Result: cd xsens_node && make -j5 && cd .. && cd seabee3_driver && make -j5 && cd .."
	echo ""
	echo "Example2: bash make.sh --command \"rosmake @ --pjobs 5\" --target xsens_node seabee3_driver"
	echo "Result: rosmake xsens_node seabee3_driver --pjobs 5"
}

if [ $# -le 1 ]; then
	usage
	exit
fi

while [ "$1" != "" ]; do
	case $1 in
		--foreach )		shift
						mode=1
						;;
		--command )		shift
						if [ $num_commands -ge 1 ]; then
							commands="$commands && "
						fi
						while [ "$1" != "" ] && [ "$1" != "--foreach" ] && [ "$1" != "--command" ] && [ "$1" != "--target" ]; do
							commands="$commands $1"
							shift
						done
						num_commands=$((num_commands + 1))
						;;
		--target )		shift
						if [ "${1:0:2}" = "$<" ]; then
							subprocess=$1
#							echo ">$subprocess"
							subprocess=${subprocess:2:$((${#subprocess} - 4))}
#							echo ">>$subprocess"
							val=($(bash -c "$subprocess"))
#							echo ">>>${val[*]}"
							shift
							
							for item in ${val[*]}; do
#								echo "$item"
								targets[$n]=$item
								n=$(($n + 1))
							done
						else
							while [ "$1" != "" ] && [ "$1" != "--foreach" ] && [ "$1" != "--command" ] && [ "$1" != "--target" ]; do
								abs_target=$1
								findRosTarget
								targets[$n]=$rel_target
								n=$(($n + 1))
								shift
							done
						fi
						;;
		--help )		usage
						exit
						;;
		* )				usage
						exit
	esac
done

echo "targets: ${targets[*]}"
echo "commands: $commands"

for target in ${targets[*]}; do
	opt="$commands"
	opt=${opt//%/ }
	opt=${opt//$/$target}
	opt=${opt//@/${targets[@]}}
	echo "$opt"
	echo "----------"
	bash -c "source $SOURCE_PATH && $opt"
	if [ "$mode" = "0" ]; then
		break
	fi
done
