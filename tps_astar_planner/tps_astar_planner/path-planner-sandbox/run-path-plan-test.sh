path-planner-cli -g "[-2 13 90]" -s "[-2 3 90]" \
    --planner "selfdriving::TPS_Astar"  \
    -c ptgs_jackal.ini  \
    --obstacles yaml_0.yaml  \
    --planner-parameters planner-params.yaml  \
    --costmap-obstacles costmap-obstacles.yaml