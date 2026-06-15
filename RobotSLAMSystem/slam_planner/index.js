const express = require('express');
const cors = require('cors');
const fs = require('fs');
const yaml = require('js-yaml');
const app = express();
const {exec,spawn} = require('child_process');
const bodyParser = require('body-parser')

app.use(cors({
    exposedHeaders: [
        "Map-Resolution", "Origin-X", "Origin-Y"
    ]
}));

app.use(bodyParser.json());

const cmdBegin = "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: \"map\" }, pose:"
const cmdEnd = "}'"

app.post('/target', function (req, res) {
    const pose = req.body;
    const cmd = cmdBegin + JSON.stringify(pose) + cmdEnd;
    console.log(cmd)
    exec(cmd)
    res.send("done")
})

app.post('/runsh', function(req,res) {
    const cmd = 'sh ' + req.body.cmd
    console.log(cmd)
    exec(cmd)
    res.send("done")
})

let shellLocation
let locationData
let shellMap
let mapFile
let isRestarting = false;

function startListen(){
    shellLocation = spawn('sh',['./odom.sh'],{detached:true});
    shellLocation.stdout.on('data', (data) => {
        try{
            //console.log("data updated")
            locationData = yaml.safeLoad(data.toString().substring(0, data.length-4))
        } catch{}
    });
    shellLocation.stderr.on('data', (data) => {
        console.error(data);
    });

    shellMap = spawn('sh',['./save.sh'],{detached:true});
    shellMap.stdout.on('data', (data) => {
        //console.log("map updated")
        dataString = data.toString();
        if (dataString.length<10){
            try {
                const mapCount = parseInt(dataString);
                info = yaml.safeLoad(fs.readFileSync(__dirname + '/Files/' + 'map' + mapCount + '.yaml', 'utf8'));
                path = __dirname + '/Files/map' + mapCount + '.pgm' // 模拟选项
        
                mapFile = { info, path }
            } catch{
                mapFile = null;
            }
        }
    });
    shellMap.stderr.on('data', (data) => {
        console.error(data);
    });
}

startListen()

app.post('/restart', function(req,res){
    
    if(!isRestarting){
        isRestarting = true;
        const restartCMD = exec('sh ./start.sh')
        restartCMD.stdout.on('data',(data)=>{
            if(data.indexOf("SUCCESS")>=0){
                console.log("restarting")
                try {
                    spawn("sh",["-c","kill -INT -"+shellLocation.pid])
                    spawn("sh",["-c","kill -INT -"+shellMap.pid])
                    startListen()
                    res.send("restarted")
                    isRestarting = false
                } catch { }
            }
        })
    } else {
        res.statusCode = 404;
        res.write('restart in process');
        res.end();
    }

})

app.get('/location', function (req, res) {
    if (locationData&&!isRestarting) {
        res.send(locationData.pose.pose)
    } else {
        res.statusCode = 404;
        res.write('location not available');
        res.end();
    }
})

app.get('/map', function (req, res) {

    if (mapFile) {
        res.setHeader("Map-Resolution", mapFile.info.resolution);
        res.setHeader("Origin-X", mapFile.info.origin[0]);
        res.setHeader("Origin-Y", mapFile.info.origin[1]);
        res.sendFile(mapFile.path);
    }
    else {
        res.statusCode = 404;
        res.write('map not available');
        res.end();
    }
});

app.listen(5000);
