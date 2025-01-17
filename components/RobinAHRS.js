//=====================================================================================================
// RobinAHRS.js
//
// Copyright 2024 Foster Electric Co., Ltd.
// Released under the MIT license
//=====================================================================================================


class Data_Storage {
    constructor(fs) {
        this.fs   = fs
        this.size = Math.round(this.fs*100.0)
        this.header = [[
            "IMU Gx", "IMU Gy", "IMU Gz",
            "IMU Ax", "IMU Ay", "IMU Az",
            "Acc X",  "Acc Y",  "Acc Z",
            "Euler X",  "Euler Y",  "Euler Z",
            "Quat W",  "Quat X",  "Quat Y",  "Quat Z", 
            "qHome W", "qHome X", "qHome Y", "qHome Z", 
            "qZero W", "qZero X", "qZero Y", "qZero Z", 
        ]]
        this.data = [[]]
    }
    update(imu_g, imu_a, accel, euler, quat, qHome, qZero) {
        let tmp = [
            imu_g[0], imu_g[1], imu_g[2], 
            imu_a[0], imu_a[1], imu_a[2], 
            accel[0], accel[1], accel[2], 
            euler[0], euler[1], euler[2],
            quat[0],  quat[1],  quat[2],  quat[3],
            qHome[0], qHome[1], qHome[2], qHome[3],
            qZero[0], qZero[1], qZero[2], qZero[3],
        ]
        this.data.push(tmp)
        if (this.data.length > this.size) {
            this.data.shift()
        }
    }
    export() {
        return this.header.concat(this.data)
    }
}

class IMU_Filter {
    constructor(fs) {
        this.fs   = fs
        this.size = Math.round(this.fs*7.5)
        this.g = [[0.0, 0.0, 0.0]]
        this.g_ofs = [0.0, 0.0, 0.0]
        this.g_ofs_hist = []
        this.a = [[0.0, 0.0, 0.0]]
        this.a_pow_hist = []

        this.timer = setInterval(() => {
            let tmp = [
                this.g.reduce((g,val)=>g+val[0], 0)/this.g.length,
                this.g.reduce((g,val)=>g+val[1], 0)/this.g.length,
                this.g.reduce((g,val)=>g+val[2], 0)/this.g.length,
            ]
            let a_ave  = this.a_pow_hist.reduce((a,val)=>a+val, 0)/this.a_pow_hist.length
            let a_move = this.a_pow_hist.reduce((a,val)=>a+Math.abs(val-a_ave), 0).toFixed(5)
            let g_move = this.g.reduce((a,val)=>a+Math.abs(val[0])+Math.abs(val[1])+Math.abs(val[2]), 0).toFixed(5)

            //console.log(tmp.map((x)=>x.toFixed(5)), this.g_ofs.map((x)=>x.toFixed(5)), g_move, a_move)
        }, 2000)
    }
    get gyro() {
        return this.g[this.g.length-1]
    }
    get accl() {
        return this.a[this.a.length-1]
    }
    get total_accl() {
        return this.a_pow_hist[this.a_pow_hist.length-1]
    }

    update(g0, g1, g2, a0, a1, a2) {
        g0 = g0/this.fs - this.g_ofs[0]
        g1 = g1/this.fs - this.g_ofs[1]
        g2 = g2/this.fs - this.g_ofs[2]

        this.g.push([g0, g1, g2])
        this.a.push([a0, a1, a2])
        this.a_pow_hist.push(Math.sqrt((a0*a0) + (a1*a1) + (a2*a2)))

        if (this.a.length > this.size) {
            this.a.shift()
        }
        if (this.a_pow_hist.length > this.size) {
            this.a_pow_hist.shift()
        }
        if (this.g.length > this.size) {
            this.g.shift()

            let a_ave  = this.a_pow_hist.reduce((a,val)=>a+val, 0)/this.a.length
            let a_move = this.a_pow_hist.reduce((a,val)=>a+Math.abs(val-a_ave), 0)
            let g_move = this.g.reduce((g,val)=>g+Math.abs(val[0])+Math.abs(val[1])+Math.abs(val[2]), 0)

            if ((a_move < 0.3) && (g_move < 0.2)) {
                this.g_ofs_hist.push([
                    this.g_ofs[0] + (this.g.reduce((g,val)=>g+val[0], 0)/this.g.length),
                    this.g_ofs[1] + (this.g.reduce((g,val)=>g+val[1], 0)/this.g.length),
                    this.g_ofs[2] + (this.g.reduce((g,val)=>g+val[2], 0)/this.g.length),
                ])
                if (this.g_ofs_hist.length > 5) {
                    this.g_ofs_hist.shift()
                }
                this.g_ofs[0] = this.g_ofs_hist.reduce((g,val)=>g+val[0], 0)/this.g_ofs_hist.length,
                this.g_ofs[1] = this.g_ofs_hist.reduce((g,val)=>g+val[1], 0)/this.g_ofs_hist.length,
                this.g_ofs[2] = this.g_ofs_hist.reduce((g,val)=>g+val[2], 0)/this.g_ofs_hist.length,

                this.g = [[0.0, 0.0, 0.0]]
                console.log("Gyro offset updated. : ", this.g_ofs.map((x)=>x.toFixed(5)))
            }
        }
    }

    set_gyro_ofs(ofs) {
        if (ofs === undefined) {
            ofs = [this.g_ofs[0], this.g_ofs[1], this.g_ofs[2]]

            ofs[0] += (this.g.reduce((g,val)=>g+val[0], 0)/this.g.length)
            ofs[1] += (this.g.reduce((g,val)=>g+val[1], 0)/this.g.length)
            ofs[2] += (this.g.reduce((g,val)=>g+val[2], 0)/this.g.length)
        }

        console.log('*** SET GYRO OFFSET *** : ' + ofs)
        this.g_ofs = ofs
        this.g_ofs_hist = []
    }
}

class RobinAHRS {
    constructor(fs) {
        this.fs = fs
        this.quat  = [1.0, 0.0, 0.0, 0.0]
        this.qZero = [1.0, 0.0, 0.0, 0.0]
        this.qHome = [NaN, NaN, NaN, NaN]
        this.gravity = [0.0, 0.0, 1.0]
        this.accl    = [0.0, 0.0, 1.0]
        this.imu     = new IMU_Filter(this.fs)
        this.data_log = new Data_Storage(this.fs)

        this.norm = function(vec) {
            let tmp = 0
            for (let i=0; i<vec.length; i++) {
                tmp += (vec[i]*vec[i])
            }
            tmp = Math.sqrt(tmp)
            return vec.map((d) => (d/tmp))
        }
        this.to_euler = function(q) {       //Rotation Order : Z->Y->X
            let x =  Math.atan2((2*q[0]*q[1] + 2*q[2]*q[3]), (2*q[0]*q[0] + 2*q[3]*q[3] -1))
            let y =  Math.asin ((2*q[0]*q[2] - 2*q[1]*q[3]))
            let z =  Math.atan2((2*q[0]*q[3] + 2*q[1]*q[2]), (2*q[0]*q[0] + 2*q[1]*q[1] -1))
            return [x, y, z]   
        }
        this.prod = function(q, p) {
            let w = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]
            let x = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2]
            let y = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1]
            let z = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0]
            return [w, x, y, z]
        }
        this.rotate = function(q, p) {
            let qConj = [q[0], -q[1], -q[2], -q[3]]
            let tmp
            tmp = this.prod(q, p)
            tmp = this.prod(tmp, qConj)
            return tmp
        }
        this.calc_qHome = function(accl) {
            let xyz = this.norm(accl)
            let tgt = this.norm([ 0.0, 0.0, 1.0])

            // Compute the rotation axis (cross product)
            let rotation_axis = this.norm([
                xyz[1]*tgt[2] - xyz[2]*tgt[1],
                xyz[2]*tgt[0] - xyz[0]*tgt[2],
                xyz[0]*tgt[1] - xyz[1]*tgt[0],
            ])
            let cos_theta = xyz[0]*tgt[0] + xyz[1]*tgt[1] + xyz[2]*tgt[2]     // Angle between the vectors (dot product)
            let angle_radians = Math.acos(cos_theta)

            let w = Math.cos(angle_radians / 2)
            let x = rotation_axis[0] * Math.sin(angle_radians / 2)
            let y = rotation_axis[1] * Math.sin(angle_radians / 2)
            let z = rotation_axis[2] * Math.sin(angle_radians / 2)
            return [w, x, y, z]
        }
    }
    get Quat() {
        return this.prod(this.quat, this.qZero)
    }
    get XYZ() {
        //return [this.accl[0]-this.gravity[0], this.accl[1]-this.gravity[1], this.accl[2]-this.gravity[2]]
        return this.to_euler(this.prod(this.qZero, this.quat))
        //return this.imu.gyro
    }
    get BodyBalance() {
        return this.accl
    }

    update(g0, g1, g2, a0, a1, a2) {
        this.imu.update(g0, g1, g2, a0, a1, a2)

        if(this.qHome.includes(NaN) || this.qZero.includes(NaN)) {
            this.qZero = [this.quat[0], -this.quat[1], -this.quat[2], -this.quat[3]]
            this.qHome = this.calc_qHome(this.imu.accl)
        }

        let gyro = [this.imu.gyro[0], this.imu.gyro[1], this.imu.gyro[2]]
        gyro = this.rotate(this.qHome, [0, gyro[0], gyro[1], gyro[2]]).slice(1,4)
        gyro = this.rotate(this.qZero, [0, gyro[0], gyro[1], gyro[2]]).slice(1,4)

        let qDot = this.prod(this.quat, [0, gyro[0], gyro[1], gyro[2]])
        let tmp  = [0,0,0,0]
        for (let i=0; i<tmp.length; i++) {
            tmp[i] = this.quat[i] + (qDot[i] * 0.5)
        }
        this.quat = this.norm(tmp)

        let accl = [this.imu.accl[0], this.imu.accl[1], this.imu.accl[2]]
        accl = this.rotate(this.qHome, [0, accl[0], accl[1], accl[2]]).slice(1,4)
        accl = this.rotate(this.qZero, [0, accl[0], accl[1], accl[2]]).slice(1,4)

        this.accl = this.rotate(this.quat, [0, accl[0], accl[1], accl[2]]).slice(1,4)

        this.data_log.update([g0, g1, g2], [a0, a1, a2], this.accl, this.to_euler(this.prod(this.qHome, this.quat)), this.quat, this.qHome, this.qZero)
    }

    reposition() {
        this.qZero = [NaN, NaN, NaN, NaN]
        this.qHome = [NaN, NaN, NaN, NaN]
        
        //let accl = [0, this.accl[0], this.accl[1], this.accl[2]]
        //accl = this.rotate(this.qHome, accl)
        //this.gravity = [accl[0], accl[1], accl[2]]
    }

    history() {
        return this.data_log.export()
    }

}
