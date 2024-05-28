rm -f rrmgreedy.xml rrmgreedy-before.xml rrmgreedy-after.xml
./ns3 run rrm-greedy-test
# sed -i 's/XMLNEWLINE/\&#xA;/' rrmgreedy.xml
sed -i 's/XMLNEWLINE/\&#xA;/g' rrmgreedy-before.xml
sed -i 's/XMLNEWLINE/\&#xA;/g' rrmgreedy-after.xml
