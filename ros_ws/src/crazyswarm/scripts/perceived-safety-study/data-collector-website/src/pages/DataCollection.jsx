import { Box, Typography } from "@mui/material";
import InputBtn from "components/InputBtn";

import { doc, onSnapshot } from "firebase/firestore";
import { useEffect, useState } from "react";
import { db } from "utils/fb";

const DataCollection = () => {
  const [perceivedSafety, setPerceivedSafety] = useState(99);

  useEffect(() => {
    return onSnapshot(doc(db, "participantData", "1"), (doc) => {
      setPerceivedSafety(doc.data().perceivedSafety);
    });
  }, []);

  const scoreToComment = {
    "-3": "Too unsafe",
    0: "Perfect",
    3: "Too safe",
  };

  return (
    <div>
      {-3 <= perceivedSafety && perceivedSafety <= 3 ? (
        <>
          <Typography variant="h5" sx={{ textAlign: "center", mt: "175px" }}>
            Your answer:
          </Typography>
          <Typography
            variant="h5"
            sx={{ textAlign: "center", fontSize: "100px" }}
          >
            {perceivedSafety}{" "}
            {scoreToComment[perceivedSafety] &&
              `(${scoreToComment[perceivedSafety]})`}
          </Typography>
        </>
      ) : (
        <Box
          sx={{
            display: "flex",
            alignItems: "center",
            flexDirection: "column",
            justifyContent: "center",
            pt: "10px",
            rowGap: "20px",
            flexWrap: "wrap",
            width: "100%",
          }}
        >
          <Typography
            variant="h4"
            sx={{
              textAlign: "center",
              fontSize: "30px",
            }}
          >
            How did you perceive the most recent trajectory?
          </Typography>

          <InputBtn number={-3} text="too unsafe" />
          <InputBtn number={-2} />
          <InputBtn number={-1} />
          <InputBtn number={0} text="perfect" />
          <InputBtn number={1} />
          <InputBtn number={2} />
          <InputBtn number={3} text="too safe" />
        </Box>
      )}
    </div>
  );
};

export default DataCollection;
